import asyncio
import math
import flight
from comms import state
from utils import clamp, calculate_distance

# =====================================================================
# KRTI 2024 MASTER SEQUENCE (PURE ODOMETRY + YOLO STEERING)
# =====================================================================

class MissionContext:
    def __init__(self):
        self.state_phase = "IDLE"
        
        # Dead Reckoning Anchors
        self.blind_start_x = 0.0
        self.blind_start_y = 0.0
        self.dist_flown = 0.0
        
        # PID Tuning
        self.kp_yaw = 0.005

class BaseState:
    async def execute(self, drone, ctx):
        pass

# ---------------------------------------------------------
# 1. DOUBLE GATE
# ---------------------------------------------------------
class FindDoubleGate(BaseState):
    def __init__(self):
        self.state_distance = 4.0 # TOTAL JARAK DARI WP2 (ARUCO 1) SAMPAI NEMBUS DOUBLE GATE
        self.initialized = False
        
    async def execute(self, drone, ctx):
        if not self.initialized:
            ctx.blind_start_x = state.x
            ctx.blind_start_y = state.y
            self.initialized = True

        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] DOUBLE GATE SELESAI ({self.state_distance}m). Lanjut Drop Box...")
            ctx.state_phase = "FIND_DROPBOX"
            self.initialized = False
            return

        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        yaw_cmd = 0.0
        fwd_cmd = 0.8

        if front_status == "LOCKED" and front_class == "DoubleGate":
            yaw_cmd = front_err_x * ctx.kp_yaw
            if abs(front_err_x) > 50:
                fwd_cmd = 0.3 # Ngerem dikit pas nikung
                
        print(f"[AUTOPILOT] [DOUBLE GATE] Terbang: {ctx.dist_flown:.2f}/{self.state_distance}m | Yaw: {yaw_cmd:.2f}")
        await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)

# ---------------------------------------------------------
# 2. DROP BOX (MEDKIT)
# ---------------------------------------------------------
class FindDropBox(BaseState):
    def __init__(self):
        self.state_distance = 3.0 # JARAK DARI SETELAH DOUBLE GATE KE TITIK DROP BOX
        self.initialized = False
        self.timeout_counter = 0
        
    async def execute(self, drone, ctx):
        if not self.initialized:
            ctx.blind_start_x = state.x
            ctx.blind_start_y = state.y
            self.initialized = True
            self.timeout_counter = 0

        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        if ctx.dist_flown > self.state_distance:
            # STOP DAN DROP!
            print("[AUTOPILOT] PAS DI ATAS DROP BOX! CEKREK SERVO DIBUKA! 💣")
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
            
            # Anggap kita nunggu 3 detik biar lu (manusia) bisa narik kotaknya buat munculin Aruco 2
            self.timeout_counter += 1
            if self.timeout_counter > 30: # 3 detik (10Hz)
                print("[AUTOPILOT] Jeda Selesai. Lanjut nyari Aruco 2...")
                ctx.state_phase = "FIND_ARUCO_2"
                self.initialized = False
            return

        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        yaw_cmd = 0.0
        fwd_cmd = 0.5 # Pelan-pelan nyari kotak merah

        if front_status == "LOCKED" and front_class == "DropBox":
            yaw_cmd = front_err_x * ctx.kp_yaw
                
        print(f"[AUTOPILOT] [DROP BOX] Terbang: {ctx.dist_flown:.2f}/{self.state_distance}m | Yaw: {yaw_cmd:.2f}")
        await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)


# ---------------------------------------------------------
# 3. ARUCO 2
# ---------------------------------------------------------
class FindAruco2(BaseState):
    def __init__(self):
        self.state_distance = 1.0 # KARENA MANUSIA NARIK KOTAK, ARUCO HARUSNYA UDAH DEKET BANGET (GESER DIKIT DOANG)
        self.initialized = False
        self.timeout_counter = 0
        
    async def execute(self, drone, ctx):
        if not self.initialized:
            ctx.blind_start_x = state.x
            ctx.blind_start_y = state.y
            self.initialized = True
            self.timeout_counter = 0

        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        if ctx.dist_flown > self.state_distance:
            print("[AUTOPILOT] PAS DI ATAS ARUCO 2! SAVE CHECKPOINT...")
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
            
            self.timeout_counter += 1
            if self.timeout_counter > 20: # Hover 2 detik aja
                print("[AUTOPILOT] Lanjut ke Triple Gate...")
                ctx.state_phase = "FIND_TRIPLE_GATE"
                self.initialized = False
            return

        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        yaw_cmd = 0.0
        fwd_cmd = 0.4 # Super pelan karena deket

        if front_status == "LOCKED" and front_class == "Aruco":
            yaw_cmd = front_err_x * ctx.kp_yaw
                
        print(f"[AUTOPILOT] [ARUCO 2] Terbang: {ctx.dist_flown:.2f}/{self.state_distance}m | Yaw: {yaw_cmd:.2f}")
        await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)

# ---------------------------------------------------------
# 4. TRIPLE GATE
# ---------------------------------------------------------
class FindTripleGate(BaseState):
    def __init__(self):
        self.state_distance = 6.0 # JARAK TOTAL DARI ARUCO 2 SAMPAI NEMBUS TRIPLE GATE
        self.initialized = False
        
    async def execute(self, drone, ctx):
        if not self.initialized:
            ctx.blind_start_x = state.x
            ctx.blind_start_y = state.y
            self.initialized = True

        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] TRIPLE GATE SELESAI ({self.state_distance}m). Lanjut Aruco Finish...")
            ctx.state_phase = "FIND_ARUCO_3"
            self.initialized = False
            return

        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        yaw_cmd = 0.0
        fwd_cmd = 0.8

        if front_status == "LOCKED" and front_class == "TripleGate":
            yaw_cmd = front_err_x * ctx.kp_yaw
            if abs(front_err_x) > 50:
                fwd_cmd = 0.3
                
        print(f"[AUTOPILOT] [TRIPLE GATE] Terbang: {ctx.dist_flown:.2f}/{self.state_distance}m | Yaw: {yaw_cmd:.2f}")
        await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)

# ---------------------------------------------------------
# 5. ARUCO 3 (FINISH / LANDING)
# ---------------------------------------------------------
class FindAruco3(BaseState):
    def __init__(self):
        self.state_distance = 3.0 # JARAK DARI SETELAH TRIPLE GATE KE TITIK ARUCO FINISH
        self.initialized = False
        
    async def execute(self, drone, ctx):
        if not self.initialized:
            ctx.blind_start_x = state.x
            ctx.blind_start_y = state.y
            self.initialized = True

        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        if ctx.dist_flown > self.state_distance:
            print(f"[AUTOPILOT] TEPAT DI ATAS ARUCO FINISH! MULAI MENDARAT...")
            ctx.state_phase = "LANDING_SEQUENCE"
            self.initialized = False
            return

        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)

        z_err = -1.0 - state.z
        up_cmd = clamp(z_err * 0.5, -0.5, 0.5)

        yaw_cmd = 0.0
        fwd_cmd = 0.5

        if front_status == "LOCKED" and front_class == "Aruco":
            yaw_cmd = front_err_x * ctx.kp_yaw
                
        print(f"[AUTOPILOT] [ARUCO 3] Terbang: {ctx.dist_flown:.2f}/{self.state_distance}m | Yaw: {yaw_cmd:.2f}")
        await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)


class LandingSequence(BaseState):
    async def execute(self, drone, ctx):
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
    "FIND_ARUCO_2": FindAruco2(),
    "FIND_TRIPLE_GATE": FindTripleGate(),
    "FIND_ARUCO_3": FindAruco3(),
    "LANDING_SEQUENCE": LandingSequence(),
    "DONE": None
}
