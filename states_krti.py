import asyncio
import math
import flight
from comms import state
from utils import clamp, calculate_distance

# =====================================================================
# KRTI 2024 — PURE DEAD RECKONING (TANPA KAMERA!)
# =====================================================================
# SEMUA transisi state = JARAK (Odometry) + SUDUT (Gyro)
# YOLO = NGGAK ADA. Drone terbang buta tapi presisi.
# FAILSAFE = Berlapis: Timeout, Altitude Fence, Geofence, Kill Switch
# =====================================================================
#
# FLOW:
#   YAW_RIGHT → PUNCH_DOUBLE_GATE → PUNCH_TO_DROPBOX → DROP_MEDKIT
#   → YAW_LEFT → PUNCH_TRIPLE_GATE → PUNCH_TO_FINISH → LANDING
#
# TUNING DI LAPANGAN:
#   1. Ukur semua jarak pakai METERAN TUKANG
#   2. Update self.state_distance di setiap state
#   3. Update self.target_angle di YawRight dan YawLeft
#   4. Bench test (props off) cek telemetri
#   5. Test terbang pendek satu state dulu
# =====================================================================

# ========================
# KONFIGURASI GLOBAL
# ========================
TARGET_ALTITUDE = -1.0       # Ketinggian target (NED: negatif = naik)
MAX_ALTITUDE    = -1.8       # Batas atas (lebih negatif = lebih tinggi)
MIN_ALTITUDE    = -0.3       # Batas bawah (mendekati tanah)
GEOFENCE_RADIUS = 25.0       # Meter dari titik awal, kalau lebih = DARURAT
MISSION_TIMEOUT = 180        # Detik total misi (3 menit). Lebih = DARURAT
STATE_TIMEOUT   = 600        # Ticks per state (60 detik). Lebih = STUCK

CRUISE_SPEED    = 0.5        # m/s — Kecepatan maju utama (PELAN = PRESISI)
YAW_SPEED       = 20.0       # deg/s — Kecepatan belok (PELAN = PRESISI)

# ========================
# MISSION CONTEXT
# ========================
class MissionContext:
    def __init__(self):
        self.state_phase = "IDLE"
        
        # Dead Reckoning Anchors
        self.blind_start_x = 0.0
        self.blind_start_y = 0.0
        self.dist_flown = 0.0
        
        # Failsafe Trackers
        self.mission_start_x = 0.0
        self.mission_start_y = 0.0
        self.mission_start_time = None
        self.state_ticks = 0  # Counter per-state timeout
        
        # PID Altitude
        self.kp_alt = 0.5

class BaseState:
    async def execute(self, drone, ctx):
        pass

# =================================================================
# FAILSAFE ENGINE — Dipanggil SETIAP TICK sebelum eksekusi state
# =================================================================
async def run_failsafes(drone, ctx):
    """
    Return True kalau DARURAT (harus landing).
    Return False kalau aman (lanjut misi).
    """
    import time
    
    # --- FAILSAFE 1: ALTITUDE FENCE ---
    if state.z < MAX_ALTITUDE:
        print(f"[🚨 FAILSAFE] KETINGGIAN TERLALU TINGGI! z={state.z:.2f}m (Batas: {MAX_ALTITUDE}m). EMERGENCY LAND!")
        ctx.state_phase = "EMERGENCY_LAND"
        return True
        
    if state.z > MIN_ALTITUDE and ctx.state_phase not in ("LANDING_SEQUENCE", "EMERGENCY_LAND", "IDLE", "DONE"):
        print(f"[🚨 FAILSAFE] KETINGGIAN TERLALU RENDAH! z={state.z:.2f}m (Batas: {MIN_ALTITUDE}m). EMERGENCY LAND!")
        ctx.state_phase = "EMERGENCY_LAND"
        return True
    
    # --- FAILSAFE 2: GEOFENCE ---
    dist_from_home = calculate_distance(ctx.mission_start_x, ctx.mission_start_y, state.x, state.y)
    if dist_from_home > GEOFENCE_RADIUS:
        print(f"[🚨 FAILSAFE] GEOFENCE BREACH! Jarak dari home: {dist_from_home:.1f}m (Batas: {GEOFENCE_RADIUS}m). EMERGENCY LAND!")
        ctx.state_phase = "EMERGENCY_LAND"
        return True
    
    # --- FAILSAFE 3: MISSION TIMEOUT ---
    if ctx.mission_start_time is not None:
        elapsed = time.time() - ctx.mission_start_time
        if elapsed > MISSION_TIMEOUT:
            print(f"[🚨 FAILSAFE] MISSION TIMEOUT! Elapsed: {elapsed:.0f}s (Batas: {MISSION_TIMEOUT}s). EMERGENCY LAND!")
            ctx.state_phase = "EMERGENCY_LAND"
            return True
    
    # --- FAILSAFE 4: PER-STATE TIMEOUT ---
    ctx.state_ticks += 1
    if ctx.state_ticks > STATE_TIMEOUT:
        print(f"[🚨 FAILSAFE] STATE TIMEOUT! State '{ctx.state_phase}' stuck {ctx.state_ticks} ticks. EMERGENCY LAND!")
        ctx.state_phase = "EMERGENCY_LAND"
        return True
    
    return False  # Semua aman

# =================================================================
# HELPER
# =================================================================
def yaw_difference(current, start):
    diff = abs(current - start)
    if diff > 180:
        diff = 360 - diff
    return diff

def altitude_hold():
    """Return up_cmd untuk maintain altitude konstan."""
    z_err = TARGET_ALTITUDE - state.z
    return clamp(z_err * 0.5, -0.5, 0.5)

def reset_state_anchor(ctx):
    """Reset odometry anchor dan state tick counter."""
    ctx.blind_start_x = state.x
    ctx.blind_start_y = state.y
    ctx.dist_flown = 0.0
    ctx.state_ticks = 0

# ---------------------------------------------------------
# 0. YAW RIGHT (FULL COURSE ONLY — Belok kanan dari Aruco 1)
# ---------------------------------------------------------
class YawRight(BaseState):
    def __init__(self):
        self.target_angle = 90.0   # HARDCODE: Derajat belok kanan
        self.start_yaw = None
        
    async def execute(self, drone, ctx):
        up_cmd = altitude_hold()
        
        if self.start_yaw is None:
            self.start_yaw = state.yaw
            print(f"[AUTOPILOT] ➡️ YAW RIGHT dimulai! Heading: {self.start_yaw:.1f}°, Target: +{self.target_angle}°")
        
        diff = yaw_difference(state.yaw, self.start_yaw)
        
        if diff >= self.target_angle - 3.0:
            print(f"[AUTOPILOT] ✅ YAW RIGHT SELESAI! ({diff:.1f}°). Gas ke Double Gate!")
            self.start_yaw = None
            reset_state_anchor(ctx)
            ctx.state_phase = "PUNCH_DOUBLE_GATE"
            return
        
        print(f"[AUTOPILOT] [YAW RIGHT] {diff:.1f}° / {self.target_angle}°")
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=YAW_SPEED)

# ---------------------------------------------------------
# 1. PUNCH DOUBLE GATE (Maju lurus X meter nembus gawang)
# ---------------------------------------------------------
# FULL COURSE: Masuk dari YAW_RIGHT
# RESUME: Pilot lurusin moncong -> ketek CH6
# ---------------------------------------------------------
class PunchDoubleGate(BaseState):
    def __init__(self):
        self.state_distance = 4.0  # HARDCODE: Ukur dari titik mulai maju sampai 1m di belakang gawang
        
    async def execute(self, drone, ctx):
        up_cmd = altitude_hold()
        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] ✅ DOUBLE GATE NEMBUS! ({ctx.dist_flown:.2f}m). Gas ke Drop Box!")
            reset_state_anchor(ctx)
            ctx.state_phase = "PUNCH_TO_DROPBOX"
            return
        
        print(f"[AUTOPILOT] [DOUBLE GATE] ✈️ {ctx.dist_flown:.2f}/{self.state_distance}m")
        await flight.send_body_velocity(drone, forward_m_s=CRUISE_SPEED, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

# ---------------------------------------------------------
# 2. PUNCH TO DROP BOX (Maju lurus X meter ke kotak merah)
# ---------------------------------------------------------
class PunchToDropBox(BaseState):
    def __init__(self):
        self.state_distance = 3.0  # HARDCODE: Jarak dari belakang gawang ke titik Drop Box
        
    async def execute(self, drone, ctx):
        up_cmd = altitude_hold()
        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] ✅ SAMPAI DI DROP BOX! ({ctx.dist_flown:.2f}m). Menjatuhkan medkit...")
            reset_state_anchor(ctx)
            ctx.state_phase = "DROP_MEDKIT"
            return
        
        print(f"[AUTOPILOT] [DROP BOX] ✈️ {ctx.dist_flown:.2f}/{self.state_distance}m")
        await flight.send_body_velocity(drone, forward_m_s=CRUISE_SPEED, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

# ---------------------------------------------------------
# 3. DROP MEDKIT (Stop + Servo + Nunggu Manusia)
# ---------------------------------------------------------
class DropMedkit(BaseState):
    def __init__(self):
        self.timeout_counter = 0
        self.wait_ticks = 100      # 10 detik (10Hz) — Manusia lari pindahin kotak
        self.servo_opened = False
        
    async def execute(self, drone, ctx):
        up_cmd = altitude_hold()
        
        # HOVER DI TEMPAT
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
        
        if not self.servo_opened:
            print("[AUTOPILOT] 💣 SERVO DIBUKA! MEDKIT DIJATUHKAN!")
            # TODO: flight.set_servo(drone, channel, pwm)
            self.servo_opened = True
        
        self.timeout_counter += 1
        remaining = (self.wait_ticks - self.timeout_counter) / 10.0
        
        if self.timeout_counter % 10 == 0:  # Print tiap 1 detik aja biar nggak spam
            print(f"[AUTOPILOT] [DROP MEDKIT] ⏳ Nunggu manusia... Sisa: {remaining:.0f} detik")
        
        if self.timeout_counter > self.wait_ticks:
            print("[AUTOPILOT] ✅ Jeda selesai! Belok kiri ke Triple Gate...")
            self.timeout_counter = 0
            self.servo_opened = False
            reset_state_anchor(ctx)
            ctx.state_phase = "YAW_LEFT"  # FULL COURSE: Auto belok kiri

# ---------------------------------------------------------
# 4. YAW LEFT (FULL COURSE ONLY — Belok kiri ke Triple Gate)
# ---------------------------------------------------------
class YawLeft(BaseState):
    def __init__(self):
        self.target_angle = 30.0   # HARDCODE: Derajat belok kiri (UKUR DI LAPANGAN!)
        self.start_yaw = None
        
    async def execute(self, drone, ctx):
        up_cmd = altitude_hold()
        
        if self.start_yaw is None:
            self.start_yaw = state.yaw
            print(f"[AUTOPILOT] ⬅️ YAW LEFT dimulai! Heading: {self.start_yaw:.1f}°, Target: -{self.target_angle}°")
        
        diff = yaw_difference(state.yaw, self.start_yaw)
        
        if diff >= self.target_angle - 3.0:
            print(f"[AUTOPILOT] ✅ YAW LEFT SELESAI! ({diff:.1f}°). Gas ke Triple Gate!")
            self.start_yaw = None
            reset_state_anchor(ctx)
            ctx.state_phase = "PUNCH_TRIPLE_GATE"
            return
        
        print(f"[AUTOPILOT] [YAW LEFT] {diff:.1f}° / {self.target_angle}°")
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=-YAW_SPEED)

# ---------------------------------------------------------
# 5. PUNCH TRIPLE GATE (Maju lurus X meter nembus gawang)
# ---------------------------------------------------------
# FULL COURSE: Masuk dari YAW_LEFT
# RESUME: Pilot lurusin moncong -> ketek CH6
# ---------------------------------------------------------
class PunchTripleGate(BaseState):
    def __init__(self):
        self.state_distance = 6.0  # HARDCODE: Jarak dari titik mulai sampai 1m di belakang Triple Gate
        
    async def execute(self, drone, ctx):
        up_cmd = altitude_hold()
        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] ✅ TRIPLE GATE NEMBUS! ({ctx.dist_flown:.2f}m). Gas ke Finish!")
            reset_state_anchor(ctx)
            ctx.state_phase = "PUNCH_TO_FINISH"
            return
        
        print(f"[AUTOPILOT] [TRIPLE GATE] ✈️ {ctx.dist_flown:.2f}/{self.state_distance}m")
        await flight.send_body_velocity(drone, forward_m_s=CRUISE_SPEED, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

# ---------------------------------------------------------
# 6. PUNCH TO FINISH (Maju lurus X meter ke titik Aruco 3)
# ---------------------------------------------------------
class PunchToFinish(BaseState):
    def __init__(self):
        self.state_distance = 3.0  # HARDCODE: Jarak dari belakang Triple Gate ke titik landing
        
    async def execute(self, drone, ctx):
        up_cmd = altitude_hold()
        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] ✅ SAMPAI DI TITIK FINISH! ({ctx.dist_flown:.2f}m). MENDARAT...")
            ctx.state_phase = "LANDING_SEQUENCE"
            return
        
        print(f"[AUTOPILOT] [FINISH] ✈️ {ctx.dist_flown:.2f}/{self.state_distance}m")
        await flight.send_body_velocity(drone, forward_m_s=CRUISE_SPEED, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

# ---------------------------------------------------------
# 7. LANDING
# ---------------------------------------------------------
class LandingSequence(BaseState):
    async def execute(self, drone, ctx):
        print(f"[AUTOPILOT] 🛬 LANDING! Altitude: {state.z:.2f}m")
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.4, yaw_deg_s=0.0)
        
        if state.z > -0.15:
            print("[AUTOPILOT] 🏁 MISI SELESAI! MATIKAN MESIN!")
            ctx.state_phase = "DONE"

# ---------------------------------------------------------
# 8. EMERGENCY LAND (Failsafe triggered)
# ---------------------------------------------------------
class EmergencyLand(BaseState):
    async def execute(self, drone, ctx):
        print(f"[🚨 EMERGENCY LAND] TURUN DARURAT! z={state.z:.2f}m")
        # Turun agresif tanpa maju/mundur/kiri/kanan
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.6, yaw_deg_s=0.0)
        
        if state.z > -0.15:
            print("[🚨 EMERGENCY LAND] MENDARAT DARURAT SELESAI. MATIKAN MESIN!")
            ctx.state_phase = "DONE"

# =====================================================================
# REGISTRY STATE 
# =====================================================================
STATE_REGISTRY = {
    "IDLE": None,
    "YAW_RIGHT": YawRight(),
    "PUNCH_DOUBLE_GATE": PunchDoubleGate(),
    "PUNCH_TO_DROPBOX": PunchToDropBox(),
    "DROP_MEDKIT": DropMedkit(),
    "YAW_LEFT": YawLeft(),
    "PUNCH_TRIPLE_GATE": PunchTripleGate(),
    "PUNCH_TO_FINISH": PunchToFinish(),
    "LANDING_SEQUENCE": LandingSequence(),
    "EMERGENCY_LAND": EmergencyLand(),
    "DONE": None
}
