import asyncio
import math
import flight
from comms import state
from utils import clamp, calculate_distance

# =====================================================================
# KRTI 2024 MASTER SEQUENCE (1 CAMERA + ODOMETRY DEAD RECKONING)
# =====================================================================

class MissionContext:
    def __init__(self):
        self.state_phase = "IDLE"
        self.timeout_counter = 0
        self.has_seen_target = False
        
        # Dead Reckoning Anchors
        self.blind_start_x = 0.0
        self.blind_start_y = 0.0
        self.dist_flown = 0.0
        
        # PID Tuning
        self.kp_yaw = 0.005
        self.kp_up = 0.005
        
        self.target_locked_area = 0

class BaseState:
    async def execute(self, drone, ctx):
        pass

# ---------------------------------------------------------
# 1. DOUBLE GATE
# ---------------------------------------------------------
class FindDoubleGate(BaseState):
    def __init__(self):
        self.punch_distance = 4.0 # BERAPA METER DARI AWAL NGE-LOCK SAMPAI NEMBUS GAWANG
        
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        z_err = -1.5 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        if front_status == "LOCKED" and front_class == "DoubleGate":
            if not ctx.has_seen_target:
                # Kunci koordinat AWAL pas pertama kali ngelihat gawang!
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = True
                
            yaw_cmd = front_err_x * ctx.kp_yaw
            fwd_cmd = 0.8
            
            # Kalau miring banget, rem bentar buat lurusin moncong
            if abs(front_err_x) > 50:
                fwd_cmd = 0.2
                
            ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
            print(f"[AUTOPILOT] [DOUBLE GATE] Centering... Terbang: {ctx.dist_flown:.2f}/{self.punch_distance}m")
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
        else:
            if ctx.has_seen_target:
                # Kalau hilang di tengah jalan, tetep hitung jarak dari awal nge-lock
                ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
                if ctx.dist_flown > self.punch_distance:
                    print(f"[AUTOPILOT] NEMBUS DOUBLE GATE! (Jarak {ctx.dist_flown:.2f}m tercapai). Lanjut Drop Box...")
                    ctx.state_phase = "FIND_DROPBOX"
                    ctx.has_seen_target = False
                    return
                else:
                    # ACTIVE LIDAR ANTI-DRIFT (Kalo jarak lidar di bawah 2 meter, berarti ngelewatin tiang gawang)
                    strafe_cmd = 0.0
                    if state.lidar_left < 2.0 or state.lidar_right < 2.0:
                        strafe_cmd = (state.lidar_right - state.lidar_left) * 0.1
                        strafe_cmd = clamp(strafe_cmd, -0.3, 0.3)
                        
                    print(f"[AUTOPILOT] Gawang hilang! Blind punch sisa jarak... ({ctx.dist_flown:.2f}/{self.punch_distance}m). Anti-Drift: {strafe_cmd:.2f}")
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
                    return
            
            print("[AUTOPILOT] Nyari Double Gate... Terbang lurus pelan.")
            await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

# ---------------------------------------------------------
# 2. DROP BOX (MEDKIT)
# ---------------------------------------------------------
class FindDropBox(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_area = state.target_front.get("area", 0)

        z_err = -1.5 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        if front_status == "LOCKED" and front_class == "DropBox":
            ctx.has_seen_target = True
            ctx.timeout_counter = 0
            ctx.target_locked_area = front_area
            
            yaw_cmd = front_err_x * ctx.kp_yaw
            print(f"[AUTOPILOT] [DROP BOX] Terdeteksi! Mendekat... Area: {front_area}")
            await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
        else:
            if ctx.has_seen_target:
                # Kotak merah hilang masuk ke kolong perut drone!
                if ctx.target_locked_area > 20000: # Settingan lu: area seberapa besar sebelum hilang
                    print("[AUTOPILOT] DROP BOX MASUK BLIND SPOT! Transisi ke Dead Reckoning Drop...")
                    ctx.blind_start_x = state.x
                    ctx.blind_start_y = state.y
                    ctx.state_phase = "DEAD_RECKONING_DROP"
                    ctx.has_seen_target = False
                    return
            
            print("[AUTOPILOT] Nyari Drop Box... Terbang lurus pelan.")
            await flight.send_body_velocity(drone, forward_m_s=0.4, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

class DeadReckoningDrop(BaseState):
    def __init__(self, next_phase):
        self.next_phase = next_phase
        self.drop_distance = 1.0 # BRP METER DR HILANG SAMPE PAS DI PERUT (HARDCODE SINI)
        
    async def execute(self, drone, ctx):
        z_err = -1.5 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
        
        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown < self.drop_distance:
            print(f"[AUTOPILOT] [BLIND DROP] Maju presisi... Jarak: {ctx.dist_flown:.2f} / {self.drop_distance} m")
            await flight.send_body_velocity(drone, forward_m_s=0.4, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
        else:
            # STOP DAN DROP!
            print("[AUTOPILOT] PAS DI ATAS DROP BOX! CEKREK SERVO DIBUKA! 💣")
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
            
            # Anggap kita nunggu 3 detik biar lu (manusia) bisa narik kotaknya buat munculin Aruco 2
            ctx.timeout_counter += 1
            if ctx.timeout_counter > 30: # 3 detik (10Hz)
                print("[AUTOPILOT] Jeda Selesai. Lanjut nyari Aruco 2...")
                ctx.state_phase = self.next_phase
                ctx.timeout_counter = 0

# ---------------------------------------------------------
# 3. ARUCO 2
# ---------------------------------------------------------
class FindAruco2(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_area = state.target_front.get("area", 0)

        z_err = -1.5 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        if front_status == "LOCKED" and front_class == "Aruco":
            ctx.has_seen_target = True
            ctx.timeout_counter = 0
            ctx.target_locked_area = front_area
            
            yaw_cmd = front_err_x * ctx.kp_yaw
            print(f"[AUTOPILOT] [ARUCO 2] Terdeteksi! Mendekat... Area: {front_area}")
            await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
        else:
            if ctx.has_seen_target:
                # Aruco hilang masuk kolong
                if ctx.target_locked_area > 20000:
                    print("[AUTOPILOT] ARUCO 2 MASUK BLIND SPOT! Transisi ke Dead Reckoning Hover...")
                    ctx.blind_start_x = state.x
                    ctx.blind_start_y = state.y
                    ctx.state_phase = "DEAD_RECKONING_HOVER"
                    ctx.has_seen_target = False
                    return
            
            await flight.send_body_velocity(drone, forward_m_s=0.4, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

class DeadReckoningHover(BaseState):
    def __init__(self, next_phase):
        self.next_phase = next_phase
        self.drop_distance = 1.0 # Samain kayak drop box (jarak dari kamera ke perut)
        
    async def execute(self, drone, ctx):
        z_err = -1.5 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
        
        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown < self.drop_distance:
            print(f"[AUTOPILOT] [BLIND HOVER] Maju presisi... Jarak: {ctx.dist_flown:.2f} / {self.drop_distance} m")
            await flight.send_body_velocity(drone, forward_m_s=0.4, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
        else:
            # STOP DAN HOVER DI ATAS ARUCO!
            print("[AUTOPILOT] PAS DI ATAS ARUCO 2! SAVE CHECKPOINT...")
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
            
            ctx.timeout_counter += 1
            if ctx.timeout_counter > 20: # Hover 2 detik aja
                print("[AUTOPILOT] Lanjut ke Triple Gate...")
                ctx.state_phase = self.next_phase
                ctx.timeout_counter = 0

# ---------------------------------------------------------
# 4. TRIPLE GATE
# ---------------------------------------------------------
class FindTripleGate(BaseState):
    def __init__(self):
        self.punch_distance = 5.0 # JARAK NEMBUS TRIPLE GATE DARI PERTAMA KALI NGE-LOCK
        
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        z_err = -1.5 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        if front_status == "LOCKED" and front_class == "TripleGate":
            if not ctx.has_seen_target:
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = True
            
            yaw_cmd = front_err_x * ctx.kp_yaw
            fwd_cmd = 0.8
            if abs(front_err_x) > 50:
                fwd_cmd = 0.2
                
            ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
            print(f"[AUTOPILOT] [TRIPLE GATE] Centering... Terbang: {ctx.dist_flown:.2f}/{self.punch_distance}m")
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
        else:
            if ctx.has_seen_target:
                ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
                if ctx.dist_flown > self.punch_distance:
                    print(f"[AUTOPILOT] NEMBUS TRIPLE GATE! (Jarak {ctx.dist_flown:.2f}m). Lanjut Aruco Finish...")
                    ctx.state_phase = "FIND_ARUCO_3"
                    ctx.has_seen_target = False
                    return
                else:
                    # ACTIVE LIDAR ANTI-DRIFT 
                    strafe_cmd = 0.0
                    if state.lidar_left < 2.0 or state.lidar_right < 2.0:
                        strafe_cmd = (state.lidar_right - state.lidar_left) * 0.1
                        strafe_cmd = clamp(strafe_cmd, -0.3, 0.3)
                        
                    print(f"[AUTOPILOT] Gawang hilang! Blind punch sisa jarak... ({ctx.dist_flown:.2f}/{self.punch_distance}m). Anti-Drift: {strafe_cmd:.2f}")
                    await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
                    return
            
            print("[AUTOPILOT] Nyari Triple Gate... Terbang lurus pelan.")
            await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

# ---------------------------------------------------------
# 5. ARUCO 3 (FINISH / LANDING)
# ---------------------------------------------------------
class FindAruco3(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_area = state.target_front.get("area", 0)

        z_err = -1.5 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        if front_status == "LOCKED" and front_class == "Aruco":
            ctx.has_seen_target = True
            ctx.timeout_counter = 0
            ctx.target_locked_area = front_area
            
            yaw_cmd = front_err_x * ctx.kp_yaw
            print(f"[AUTOPILOT] [ARUCO 3 - FINISH] Terdeteksi! Mendekat... Area: {front_area}")
            await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
        else:
            if ctx.has_seen_target:
                if ctx.target_locked_area > 20000:
                    print("[AUTOPILOT] ARUCO 3 MASUK BLIND SPOT! Transisi ke Pendaratan Presisi...")
                    ctx.blind_start_x = state.x
                    ctx.blind_start_y = state.y
                    ctx.state_phase = "LANDING_SEQUENCE"
                    ctx.has_seen_target = False
                    return
            
            await flight.send_body_velocity(drone, forward_m_s=0.4, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)

class LandingSequence(BaseState):
    def __init__(self):
        self.drop_distance = 1.0 # Jarak blind punch sblm landing
        
    async def execute(self, drone, ctx):
        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown < self.drop_distance:
            z_err = -1.5 - state.z
            up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
            print(f"[AUTOPILOT] [LANDING ALIGN] Maju presisi... Jarak: {ctx.dist_flown:.2f} / {self.drop_distance} m")
            await flight.send_body_velocity(drone, forward_m_s=0.4, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
        else:
            # STOP DAN TURUN!
            print(f"[AUTOPILOT] LANDING SEKARANG! Altitude: {state.z:.2f}m")
            # Positif Z artinya turun di NED frame
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.5, yaw_deg_s=0.0)
            
            if state.z > -0.1: # Udah nyentuh tanah
                print("[AUTOPILOT] MISI SELESAI. MATIKAN MESIN!")
                ctx.state_phase = "DONE"

# =====================================================================
# REGISTRY STATE 
# =====================================================================
STATE_REGISTRY = {
    "IDLE": None,
    "FIND_DOUBLE_GATE": FindDoubleGate(),
    "FIND_DROPBOX": FindDropBox(),
    "DEAD_RECKONING_DROP": DeadReckoningDrop("FIND_ARUCO_2"),
    "FIND_ARUCO_2": FindAruco2(),
    "DEAD_RECKONING_HOVER": DeadReckoningHover("FIND_TRIPLE_GATE"),
    "FIND_TRIPLE_GATE": FindTripleGate(),
    "FIND_ARUCO_3": FindAruco3(),
    "LANDING_SEQUENCE": LandingSequence(),
    "DONE": None
}
