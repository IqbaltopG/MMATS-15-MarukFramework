import asyncio
import math
import flight
from comms import state
from utils import clamp, calculate_distance

# =====================================================================
# KRTI 2024 MASTER SEQUENCE (ODOMETRY-FIRST + YOLO STEERING)
# =====================================================================
# FLOW:
#   YAW_RIGHT → FIND_DOUBLE_GATE → FIND_DROPBOX → DROP_MEDKIT
#   → YAW_LEFT → FIND_TRIPLE_GATE → DEAD_RECKONING_FINISH → LANDING
# =====================================================================

class MissionContext:
    def __init__(self):
        self.state_phase = "IDLE"
        
        # Dead Reckoning Anchors
        self.blind_start_x = 0.0
        self.blind_start_y = 0.0
        self.dist_flown = 0.0
        
        # PID Tuning (TUNE DI LAPANGAN!)
        self.kp_yaw = 0.005

class BaseState:
    async def execute(self, drone, ctx):
        pass

# =================================================================
# HELPER: Hitung selisih sudut dengan wrap-around 0°↔360°
# =================================================================
def yaw_difference(current, start):
    diff = abs(current - start)
    if diff > 180:
        diff = 360 - diff
    return diff

# ---------------------------------------------------------
# 0. YAW RIGHT (~90° dari Aruco 1 ke arah Double Gate)
# ---------------------------------------------------------
class YawRight(BaseState):
    def __init__(self):
        self.target_angle = 90.0  # HARDCODE: Belok kanan berapa derajat
        self.yaw_speed = 25.0     # deg/s (pelan biar presisi)
        self.start_yaw = None
        
    async def execute(self, drone, ctx):
        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
        
        # Kunci heading awal sekali
        if self.start_yaw is None:
            self.start_yaw = state.yaw
            print(f"[AUTOPILOT] YAW RIGHT dimulai! Heading awal: {self.start_yaw:.1f}°, Target: +{self.target_angle}°")
        
        diff = yaw_difference(state.yaw, self.start_yaw)
        
        if diff >= self.target_angle - 5.0:  # Toleransi 5 derajat
            print(f"[AUTOPILOT] YAW RIGHT SELESAI! Heading: {state.yaw:.1f}° (Selisih: {diff:.1f}°). Lanjut cari Double Gate...")
            ctx.state_phase = "FIND_DOUBLE_GATE"
            self.start_yaw = None  # Reset buat pemakaian ulang
            return
        
        print(f"[AUTOPILOT] [YAW RIGHT] Muter... {diff:.1f}° / {self.target_angle}°")
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=self.yaw_speed)

# ---------------------------------------------------------
# 1. DOUBLE GATE
# ---------------------------------------------------------
class FindDoubleGate(BaseState):
    def __init__(self):
        self.state_distance = 4.0  # HARDCODE: Jarak dari titik pertama ngelihat gawang sampai nembus
        self.initialized = False
        
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        # WAIT FOR FIRST-SIGHT: Nggak gerak sampai YOLO ngelihat gawang
        if not self.initialized:
            if front_status == "LOCKED" and front_class == "DoubleGate":
                print("[AUTOPILOT] DOUBLE GATE TERDETEKSI PERTAMA KALI! Mengunci Anchor Odom...")
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                self.initialized = True
            else:
                print("[AUTOPILOT] Menunggu Double Gate masuk frame...")
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
                return

        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] DOUBLE GATE SELESAI ({self.state_distance}m). Lanjut Drop Box...")
            ctx.state_phase = "FIND_DROPBOX"
            self.initialized = False
            return

        yaw_cmd = 0.0
        fwd_cmd = 0.8

        if front_status == "LOCKED" and front_class == "DoubleGate":
            yaw_cmd = front_err_x * ctx.kp_yaw
            if abs(front_err_x) > 50:
                fwd_cmd = 0.3
                
        print(f"[AUTOPILOT] [DOUBLE GATE] Terbang: {ctx.dist_flown:.2f}/{self.state_distance}m | Yaw: {yaw_cmd:.2f}")
        await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)

# ---------------------------------------------------------
# 2. DROP BOX (NYARI KOTAK MERAH + CENTERING)
# ---------------------------------------------------------
class FindDropBox(BaseState):
    def __init__(self):
        self.state_distance = 3.0  # HARDCODE: Jarak dari setelah nembus Double Gate ke titik Drop Box
        self.initialized = False
        
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        # WAIT FOR FIRST-SIGHT
        if not self.initialized:
            if front_status == "LOCKED" and front_class == "DropBox":
                print("[AUTOPILOT] DROP BOX TERDETEKSI PERTAMA KALI! Mengunci Anchor Odom...")
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                self.initialized = True
            else:
                # Maju pelan-pelan nyari kotak merah
                print("[AUTOPILOT] Menunggu Drop Box masuk frame... Maju pelan...")
                await flight.send_body_velocity(drone, forward_m_s=0.4, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
                return

        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] SAMPAI DI ATAS DROP BOX! Transisi ke DROP_MEDKIT...")
            ctx.state_phase = "DROP_MEDKIT"
            self.initialized = False
            return

        yaw_cmd = 0.0
        fwd_cmd = 0.5

        if front_status == "LOCKED" and front_class == "DropBox":
            yaw_cmd = front_err_x * ctx.kp_yaw
                
        print(f"[AUTOPILOT] [DROP BOX] Terbang: {ctx.dist_flown:.2f}/{self.state_distance}m | Yaw: {yaw_cmd:.2f}")
        await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)

# ---------------------------------------------------------
# 3. DROP MEDKIT (STOP + BUKA SERVO + NUNGGU MANUSIA)
# ---------------------------------------------------------
class DropMedkit(BaseState):
    def __init__(self):
        self.timeout_counter = 0
        self.wait_ticks = 100  # 10 detik (10Hz) — Waktu manusia lari pindahin kotak
        self.servo_opened = False
        
    async def execute(self, drone, ctx):
        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
        
        # HOVER DI TEMPAT
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
        
        if not self.servo_opened:
            print("[AUTOPILOT] 💣 SERVO DIBUKA! MEDKIT DIJATUHKAN!")
            # TODO: Tambahin perintah servo di sini (misal: flight.set_servo(drone, channel, pwm))
            self.servo_opened = True
        
        self.timeout_counter += 1
        remaining = (self.wait_ticks - self.timeout_counter) / 10.0
        print(f"[AUTOPILOT] [DROP MEDKIT] Nunggu manusia pindahin kotak... Sisa: {remaining:.1f} detik")
        
        if self.timeout_counter > self.wait_ticks:
            print("[AUTOPILOT] Jeda selesai! Lanjut belok kiri ke Triple Gate...")
            ctx.state_phase = "YAW_LEFT"
            self.timeout_counter = 0
            self.servo_opened = False

# ---------------------------------------------------------
# 4. YAW LEFT (~30-45° dari Drop Box ke arah Triple Gate)
# ---------------------------------------------------------
class YawLeft(BaseState):
    def __init__(self):
        self.target_angle = 30.0  # HARDCODE: Belok kiri berapa derajat (UKUR DI LAPANGAN!)
        self.yaw_speed = -25.0    # Negatif = belok kiri
        self.start_yaw = None
        
    async def execute(self, drone, ctx):
        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
        
        if self.start_yaw is None:
            self.start_yaw = state.yaw
            print(f"[AUTOPILOT] YAW LEFT dimulai! Heading awal: {self.start_yaw:.1f}°, Target: -{self.target_angle}°")
        
        diff = yaw_difference(state.yaw, self.start_yaw)
        
        if diff >= self.target_angle - 5.0:
            print(f"[AUTOPILOT] YAW LEFT SELESAI! Heading: {state.yaw:.1f}° (Selisih: {diff:.1f}°). Lanjut cari Triple Gate...")
            ctx.state_phase = "FIND_TRIPLE_GATE"
            self.start_yaw = None
            return
        
        print(f"[AUTOPILOT] [YAW LEFT] Muter... {diff:.1f}° / {self.target_angle}°")
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=self.yaw_speed)

# ---------------------------------------------------------
# 5. TRIPLE GATE
# ---------------------------------------------------------
class FindTripleGate(BaseState):
    def __init__(self):
        self.state_distance = 6.0  # HARDCODE: Jarak dari titik pertama ngelihat Triple Gate sampai nembus
        self.initialized = False
        
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        if not self.initialized:
            if front_status == "LOCKED" and front_class == "TripleGate":
                print("[AUTOPILOT] TRIPLE GATE TERDETEKSI PERTAMA KALI! Mengunci Anchor Odom...")
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                self.initialized = True
            else:
                print("[AUTOPILOT] Menunggu Triple Gate masuk frame...")
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
                return

        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] TRIPLE GATE SELESAI ({self.state_distance}m). Lanjut Dead Reckoning ke Aruco Finish...")
            ctx.state_phase = "DEAD_RECKONING_FINISH"
            self.initialized = False
            return

        yaw_cmd = 0.0
        fwd_cmd = 0.8

        if front_status == "LOCKED" and front_class == "TripleGate":
            yaw_cmd = front_err_x * ctx.kp_yaw
            if abs(front_err_x) > 50:
                fwd_cmd = 0.3
                
        print(f"[AUTOPILOT] [TRIPLE GATE] Terbang: {ctx.dist_flown:.2f}/{self.state_distance}m | Yaw: {yaw_cmd:.2f}")
        await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)

# ---------------------------------------------------------
# 6. DEAD RECKONING FINISH (Maju X meter lurus ke Aruco 3)
# ---------------------------------------------------------
class DeadReckoningFinish(BaseState):
    """
    Aruco 3 itu kertas di lantai. Kamera depan horizontal di 1m susah ngelihat.
    Jadi kita SKIP deteksi YOLO, langsung Dead Reckoning maju lurus aja.
    """
    def __init__(self):
        self.state_distance = 3.0  # HARDCODE: Jarak dari belakang Triple Gate ke titik Aruco 3
        self.initialized = False
        
    async def execute(self, drone, ctx):
        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
        
        if not self.initialized:
            ctx.blind_start_x = state.x
            ctx.blind_start_y = state.y
            self.initialized = True
            print(f"[AUTOPILOT] DEAD RECKONING FINISH dimulai! Maju lurus {self.state_distance}m ke Aruco 3...")

        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] SAMPAI DI ATAS ARUCO 3! ({ctx.dist_flown:.2f}m). MULAI MENDARAT...")
            ctx.state_phase = "LANDING_SEQUENCE"
            self.initialized = False
            return

        print(f"[AUTOPILOT] [DR FINISH] Terbang lurus: {ctx.dist_flown:.2f}/{self.state_distance}m")
        await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

# ---------------------------------------------------------
# 7. LANDING
# ---------------------------------------------------------
class LandingSequence(BaseState):
    async def execute(self, drone, ctx):
        print(f"[AUTOPILOT] LANDING SEKARANG! Altitude: {state.z:.2f}m")
        # Positif down_m_s = turun (NED frame)
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.5, yaw_deg_s=0.0)
        
        if state.z > -0.15:  # Udah nyentuh tanah
            print("[AUTOPILOT] 🏁 MISI SELESAI. MATIKAN MESIN!")
            ctx.state_phase = "DONE"

# =====================================================================
# REGISTRY STATE 
# =====================================================================
STATE_REGISTRY = {
    "IDLE": None,
    "YAW_RIGHT": YawRight(),
    "FIND_DOUBLE_GATE": FindDoubleGate(),
    "FIND_DROPBOX": FindDropBox(),
    "DROP_MEDKIT": DropMedkit(),
    "YAW_LEFT": YawLeft(),
    "FIND_TRIPLE_GATE": FindTripleGate(),
    "DEAD_RECKONING_FINISH": DeadReckoningFinish(),
    "LANDING_SEQUENCE": LandingSequence(),
    "DONE": None
}
