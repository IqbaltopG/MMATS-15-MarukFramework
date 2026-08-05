import asyncio
import math
import flight
from comms import state
from utils import clamp, calculate_distance, get_stutter_creep_speed

class MissionContext:
    def __init__(self):
        self.state_phase = "BLIND_PUNCH_TAKEOFF"
        self.timeout_counter = 0
        self.has_seen_target = False
        self.last_front_err_x = 0
        self.last_front_err_y = 0
        self.last_front_area = 0
        self.altitude_locked = False
        self.blind_start_x = 0.0
        self.blind_start_y = 0.0
        self.landing_ticks = 0
        self.kp_yaw = 0.005
        self.kp_up = 0.005
        self.last_down_err_x = 0
        self.last_down_err_y = 0
        self.dist_flown = 0.0

class BaseState:
    async def execute(self, drone, ctx):
        pass

class BlindPunch_Takeoff(BaseState):
    async def execute(self, drone, ctx):
        if ctx.timeout_counter == 0:
            ctx.blind_start_x = state.x
            ctx.blind_start_y = state.y
            ctx.timeout_counter = 1
            print("[AUTOPILOT] Blind punch maju 5 meter buat ngebantu pandangan YOLO (Anti-RTF)...")
            
        ctx.dist_flown = calculate_distance(ctx.blind_start_x, ctx.blind_start_y, state.x, state.y)
        
        # Tambahin log tiap 10 tick (1 detik) biar keliatan pergerakannya
        if ctx.timeout_counter % 10 == 0:
            print(f"[AUTOPILOT] [BLIND PUNCH] Mendorong stik maju! Jarak ditempuh: {ctx.dist_flown:.2f} / 5.0m (X:{state.x:.2f}, Y:{state.y:.2f})")
        ctx.timeout_counter += 1
        
        if ctx.dist_flown > 5.0:
            ctx.timeout_counter = 0
            print("[AUTOPILOT] [BLIND PUNCH] Sukses maju 5 meter! Transisi ke CENTERING_GATE_1...")
            ctx.state_phase = "CENTERING_GATE_1"
        else:
            await flight.send_body_velocity(drone, forward_m_s=1.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

class GateCenteringBase(BaseState):
    def __init__(self, next_phase, punch_dist=3.2):
        self.next_phase = next_phase
        self.punch_dist = punch_dist
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if front_status == "LOCKED" and front_class == "Single Gate":
            ctx.has_seen_target = True
            ctx.timeout_counter = 0
            ctx.last_front_area = front_area
            ctx.last_front_err_x = front_err_x
            ctx.last_front_err_y = front_err_y
            
            # Logic: Mendekat dulu, baru Strafe untuk centering X dan Y (hindari top bar)
            if front_area < 25000:
                fwd_cmd = 0.8
                strafe_cmd = front_err_x * ctx.kp_yaw
                z_err = -0.8 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5) # Active Z=0.8m Lock
            else:
                if not ctx.altitude_locked:
                    # Belum nge-lock, kita hover dan presisi-kan
                    strafe_cmd = front_err_x * ctx.kp_yaw
                    up_cmd = (front_err_y + 150) * ctx.kp_up
                    up_cmd = clamp(up_cmd, -0.6, 0.6)
                    fwd_cmd = 0.0 # Berhenti maju buat nunggu stabil (anti-banteng)
                    
                    # Tolerance dilebarkan sedikit (30 pixel) biar ga nyangkut infinite loop
                    if abs(front_err_x) < 30 and abs(front_err_y + 150) < 30:
                        ctx.altitude_locked = True
                        print("[AUTOPILOT] [GATE] Centered! ALTITUDE LOCKED. Going Pitbull...")
                else:
                    # UDAH LOCK! Bodo amat sama error Y (Karna kalau maju drone nunduk dan bikin ilusi error Y)
                    z_err = -0.8 - state.z

                    up_cmd = clamp(z_err * 0.5, -0.5, 0.5) # Active Z=0.8m Lock
                    strafe_cmd = front_err_x * ctx.kp_yaw
                    if abs(front_err_x) > 40:
                        fwd_cmd = 0.0 # Kalo melenceng X-nya aja baru ngerem
                    else:
                        fwd_cmd = 0.8
                        
                    # ANTI-DRIFT: Kalau udah terlalu deket (Bounding Box nutupin layar), 
                    # jangan ngelakuin micro-correction nyamping karena pixel error-nya nggak akurat
                    if front_area > 100000:
                        strafe_cmd = 0.0
                        fwd_cmd = 0.8 # FORCE MAJU! Jangan ngerem gara-gara err_x > 40
                    
                print(f"[AUTOPILOT] [GATE] Centering (Area: {front_area}). Strafe: {strafe_cmd:.2f}, Z: {up_cmd:.2f}, Lock: {ctx.altitude_locked}")

            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
        else:
            if ctx.has_seen_target:
                # Syarat blind-punch: Harus udah deket banget (Area gede) atau bener-bener di tengah sebelum hilang
                if (ctx.last_front_err_y < 20 and abs(ctx.last_front_err_x) < 30 and ctx.last_front_area > 20000) or ctx.last_front_area > 150000 or ctx.timeout_counter > 0:
                    if ctx.timeout_counter == 0:
                        ctx.blind_start_x = state.x
                        ctx.blind_start_y = state.y
                    ctx.timeout_counter += 1
                else:
                    print(f"[AUTOPILOT] Gawang hilang dari jauh (Area: {ctx.last_front_area}, ErrX: {ctx.last_front_err_x}). Hovering & Climbing to 1.5m...")
                    ctx.timeout_counter = 0
            
            ctx.dist_flown = math.sqrt((state.x - ctx.blind_start_x)**2 + (state.y - ctx.blind_start_y)**2) if ctx.timeout_counter > 0 else 0
            
            if ctx.timeout_counter == 0:
                # Force drone to climb back to 1.5m while waiting for YOLO!
                z_err = -1.5 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                fwd_creep = 0.4 if not getattr(ctx, 'has_seen_target', False) else 0.0
                await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
            elif ctx.dist_flown < self.punch_dist: # PUNCH THROUGH INS
                z_err = -0.8 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                
                if ctx.timeout_counter % 10 == 0:
                    print(f"[AUTOPILOT] [GATE] Punching blind! INS Jarak: {ctx.dist_flown:.2f}/{self.punch_dist:.1f}m, Z: {up_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                print(f"[AUTOPILOT] Lolos Gate (Jarak INS: {ctx.dist_flown:.2f}m)! Transisi ke Phase selanjutnya...")
                ctx.state_phase = self.next_phase
                ctx.timeout_counter = 0
                ctx.has_seen_target = False
                ctx.altitude_locked = False


    # ---------------------------------------------------------
    # PHASE 3: FIND_ARUCO_1 (Mencari WP1 / Pad kuning)
    # ---------------------------------------------------------


class TerminalGuidance_Gate1(GateCenteringBase):
    def __init__(self):
        super().__init__("FIND_ARUCO_1", punch_dist=9.0)

class TerminalGuidance_Gate2(GateCenteringBase):
    def __init__(self):
        super().__init__("FIND_ARUCO_1", punch_dist=9.0)

class TerminalGuidance_FinalGate(GateCenteringBase):
    def __init__(self):
        super().__init__("FIND_LANDING_PAD", punch_dist=3.2)

class AcquireTarget_Aruco1(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # ---------------------------------------------------------
        if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area", "Tripple Gate"]:
            print("[AUTOPILOT] Aruco 1 (Marker/Area/Halusinasi Tripple) Terlihat di Kamera Bawah! Memulai Precision Centering...")
            ctx.state_phase = "CENTER_ARUCO_1"
            ctx.timeout_counter = 0
            ctx.has_seen_target = False
        else:
            # Kembali ke ketinggian 1.5m (state.z = -1.5)
            z_err = -1.5 - state.z
            climb_cmd = clamp(z_err * 0.5, -0.5, 0.5)
            
            if front_status == "LOCKED" and front_class == "Aruco Area":
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = True
                ctx.timeout_counter = 0
                ctx.last_front_err_x = front_err_x
                yaw_cmd = front_err_x * ctx.kp_yaw
                
                if abs(front_err_x) > 40:
                    # Target off-center! Hover and rotate to face it before pushing forward.
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
            elif ctx.has_seen_target:
                # FALLBACK MEMORY: Masuk blind spot antara kamera depan dan bawah
                ctx.dist_flown = math.sqrt((state.x - ctx.blind_start_x)**2 + (state.y - ctx.blind_start_y)**2)
                ctx.timeout_counter += 1
                
                if ctx.dist_flown > 2.5:
                    ctx.timeout_counter = 500 # Latch reverse state
                    
                if ctx.timeout_counter >= 500:
                    print(f"[AUTOPILOT] Kebablasan ArUco 1 di blind spot! Terbang mundur...")
                    await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                else:
                    # Stutter creep to level pitch and scan straight down
                    fwd_creep = 0.3 if ctx.timeout_counter % 20 < 10 else 0.0
                    await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)

    # ---------------------------------------------------------
    # PHASE 3B: CENTER_ARUCO_1 (Precision Hover)
    # ---------------------------------------------------------

class TerminalGuidance_Aruco1(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # HACK: YOLO sering halusinasi ngira Aruco jadi Tripple Gate di kamera bawah.
        # Karena Tripple Gate itu gawang vertikal, MUSTAHIL ada di lantai. 
        # Jadi kalau YOLO liat Tripple Gate di kamera bawah, anggep aja itu Aruco!
        if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area", "Tripple Gate"]:
            ctx.has_seen_target = True
            ctx.last_down_err_x = down_err_x
            ctx.last_down_err_y = down_err_y
            
            fwd_cmd = -down_err_y * 0.0015
            strafe_cmd = down_err_x * 0.0015
            
            # Limit speed so it doesn't overshoot without gimbal
            fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
            strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
            
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.0, yaw_deg_s=0.0)
            
            if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                ctx.timeout_counter += 1
                
                # Fast completion if inner marker is seen, slow fallback if only outer area is seen
                completion_threshold = 50 if down_class == "Aruco" else 150
                
                if ctx.timeout_counter > completion_threshold: 
                    print("[AUTOPILOT] Presisi WP1 Tercapai! Muter kanan nyari Triple Gate 1 (Double Gate)...")
                    ctx.state_phase = "YAW_RIGHT_TRIPLE_2"
                    ctx.timeout_counter = 0
                    ctx.has_seen_target = False
            else:
                ctx.timeout_counter = 0
        else:
            ctx.timeout_counter += 1
            if ctx.timeout_counter > 50:
                print("[AUTOPILOT] WP1 Hilang! Kembali ke FIND_ARUCO_1...")
                ctx.state_phase = "FIND_ARUCO_1"
                ctx.timeout_counter = 0
                ctx.has_seen_target = False
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
            elif ctx.has_seen_target:
                # FALLBACK MEMORY: Teruskan terbang ke memori koordinat terakhir pas lagi flicker
                fwd_cmd = clamp(-ctx.last_down_err_y * 0.0015, -0.2, 0.2)
                strafe_cmd = clamp(ctx.last_down_err_x * 0.0015, -0.2, 0.2)
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)

    # ---------------------------------------------------------
    # PHASE 4: FOLLOW_LINE_TO_WP2 (Murni Ngikutin Garis)
    # ---------------------------------------------------------

class FlyByWire_LineFollow(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        fwd_cmd = 1.0
        yaw_cmd = 0.0
        strafe_cmd = 0.0
        
        # EARLY EXIT: Jika sudah melihat Aruco 2, langsung stop ikuti garis
        # ONLY early exit if we have seen the straight line first (meaning we left Aruco 1)
        if ctx.has_seen_target and down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
            if getattr(ctx, 'is_final_line', False):
                print("[AUTOPILOT] WP3 (Aruco 3) Terlihat di Bawah! Langsung Centering...")
                ctx.state_phase = "CENTER_ARUCO_3"
            else:
                print("[AUTOPILOT] WP2 (Aruco 2) Terlihat di Bawah! Langsung Centering...")
                ctx.state_phase = "CENTER_ARUCO_2"
            ctx.timeout_counter = 0
            ctx.has_seen_target = False
            return # Skip the rest of the loop to enter new phase immediately
            

        
        if front_status == "LOCKED" and front_class == "Straight Line":
            yaw_cmd = front_err_x * ctx.kp_yaw
            
        if down_status == "LOCKED" and down_class == "Straight Line":
            strafe_cmd = down_err_x * 0.0015
            strafe_cmd = clamp(strafe_cmd, -0.3, 0.3)
            ctx.timeout_counter = 0 # Reset timeout kalau masih liat garis
            ctx.has_seen_target = True # Tandai bahwa kita udah berhasil nangkep garis
        else:
            if ctx.has_seen_target: # Cuma ngitung timeout hilang JIKA sebelumnya udah dapet garis
                ctx.timeout_counter += 1
            
        if ctx.timeout_counter > 150: # Garis hilang (RTF 30% scale)
            if getattr(ctx, 'is_final_line', False):
                print("[AUTOPILOT] Ujung Garis Final tercapai! Beralih mencari WP3 (Aruco 3)...")
                ctx.state_phase = "FIND_ARUCO_3"
            else:
                print("[AUTOPILOT] Ujung Garis WP2 tercapai! Beralih mencari Aruco 2...")
                ctx.state_phase = "FIND_ARUCO_2"
            ctx.timeout_counter = 0
            ctx.has_seen_target = False
        
        await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=global_climb_cmd, yaw_deg_s=yaw_cmd)

    # ---------------------------------------------------------
    # PHASE 4A: FIND_ARUCO_2 (Mencari Aruco setelah garis habis)
    # ---------------------------------------------------------

class AcquireTarget_Aruco2(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area", "Tripple Gate"]:
            print("[AUTOPILOT] WP2 (Aruco 2) Terlihat! Memulai Centering...")
            ctx.state_phase = "CENTER_ARUCO_2"
            ctx.timeout_counter = 0
            ctx.has_seen_target = False
        else:
            if front_status == "LOCKED" and front_class in ["Aruco Area", "Tripple Gate"]:
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = True
                ctx.timeout_counter = 0
                ctx.last_front_err_x = front_err_x
                yaw_cmd = front_err_x * ctx.kp_yaw
                
                if abs(front_err_x) > 40:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=yaw_cmd)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=yaw_cmd)
            elif ctx.has_seen_target:
                # FALLBACK MEMORY: Blind spot creep
                ctx.dist_flown = math.sqrt((state.x - ctx.blind_start_x)**2 + (state.y - ctx.blind_start_y)**2)
                ctx.timeout_counter += 1
                
                if ctx.dist_flown > 2.5:
                    ctx.timeout_counter = 500 # Latch reverse state
                    
                if ctx.timeout_counter >= 500:
                    print(f"[AUTOPILOT] Kebablasan ArUco 2 di blind spot! Terbang mundur...")
                    await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
                else:
                    # Stutter creep to level pitch and scan straight down
                    fwd_creep = 0.3 if ctx.timeout_counter % 20 < 10 else 0.0
                    await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.5, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)

    # ---------------------------------------------------------
    # PHASE 4B: CENTER_ARUCO_2 (Precision Hover)
    # ---------------------------------------------------------

class TerminalGuidance_Aruco2(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area", "Tripple Gate"]:
            ctx.has_seen_target = True
            ctx.last_down_err_x = down_err_x
            ctx.last_down_err_y = down_err_y
            
            fwd_cmd = -down_err_y * 0.0015
            strafe_cmd = down_err_x * 0.0015
            
            fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
            strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
            
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
            
            if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                ctx.timeout_counter += 1
                
                # Fast completion if inner marker is seen, slow fallback if only outer area is seen
                completion_threshold = 50 if down_class == "Aruco" else 150
                
                if ctx.timeout_counter > completion_threshold:
                    print("[AUTOPILOT] Presisi WP2 Tercapai! Mutar kanan nyari Straight Line Final...")
                    ctx.state_phase = "YAW_RIGHT_FINAL_LINE"
                    ctx.is_final_line = True
                    ctx.timeout_counter = 0
                    ctx.has_seen_target = False
            else:
                ctx.timeout_counter = 0
        else:
            ctx.timeout_counter += 1
            if ctx.timeout_counter > 50:
                print("[AUTOPILOT] WP2 Hilang! Kembali ke FIND_ARUCO_2...")
                ctx.state_phase = "FIND_ARUCO_2"
                ctx.timeout_counter = 0
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = False
            elif ctx.has_seen_target:
                # FALLBACK MEMORY: Rebound brake
                fwd_cmd = clamp(-ctx.last_down_err_y * 0.0015, -0.2, 0.2)
                strafe_cmd = clamp(ctx.last_down_err_x * 0.0015, -0.2, 0.2)
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=global_climb_cmd, yaw_deg_s=0.0)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=0.0)

    # ---------------------------------------------------------
    # PHASE 4C: YAW_LEFT_TRIPLE_1 (Belok Kiri Nyari Triple Gate)
    # ---------------------------------------------------------

class ExecuteYawSweep_FinalLine(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        down_status = state.target_down.get("status", "LOST")
        down_class = state.target_down.get("class", "none")

        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        ctx.timeout_counter += 1
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=global_climb_cmd, yaw_deg_s=25.0)
        
        if (front_status == "LOCKED" and front_class == "Straight Line") or (down_status == "LOCKED" and down_class == "Straight Line"):
            print(f"[AUTOPILOT] Straight Line Final terlihat! Memulai Line Follow...")
            ctx.state_phase = "FOLLOW_LINE_TO_WP2"
            ctx.timeout_counter = 0

class ExecuteYawSweep_Left(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        ctx.timeout_counter += 1
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=-15.0)
        if front_status == "LOCKED" and front_class == "Tripple Gate" and front_area > 10000:
            print(f"[AUTOPILOT] Triple Gate terlihat (Area: {front_area})! Memulai approach...")
            ctx.state_phase = "FIND_TRIPLE_GATE_1"
            ctx.timeout_counter = 0

    # ---------------------------------------------------------
    # PHASE 5: FIND_TRIPLE_GATE_1 (Habis WP2, masuk lorong 2 meter)
    # ---------------------------------------------------------

class AcquireTarget_TripleGate1(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if front_status == "LOCKED" and front_class == "Tripple Gate":
            ctx.has_seen_target = True
            ctx.max_front_area = max(getattr(ctx, 'max_front_area', 0), front_area)
            ctx.last_front_area = front_area
            ctx.last_front_err_y = front_err_y
            ctx.last_front_err_x = front_err_x
            yaw_cmd = front_err_x * ctx.kp_yaw
            
            if getattr(ctx, 'max_front_area', 0) < 350000:
                if abs(front_err_x) > 100:
                    fwd_cmd = 0.0 # Rem dulu biar ngadep lurus ke lubang!
                else:
                    fwd_cmd = 0.8
                z_err = -0.8 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
            else:
                yaw_cmd = 0.0 # ANTI-DRIFT: Jangan belok-belok di moncong lorong
                fwd_cmd = 0.8 # FORCE MAJU, jangan ngerem
                z_err = -0.8 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                
            ctx.timeout_counter = 0
            print(f"[AUTOPILOT] [TRIPLE GATE 2] Centering (Area: {front_area}). Fwd: {fwd_cmd}, Yaw: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
        else:
            if ctx.has_seen_target:
                ctx.timeout_counter += 1
                if ctx.timeout_counter > 5:
                    if getattr(ctx, 'max_front_area', 0) > 350000:
                        print("[AUTOPILOT] Memasuki Lorong Triple Gate 2! Berpindah ke PUNCH_TRIPLE_GATE_1")
                        ctx.state_phase = "PUNCH_TRIPLE_GATE_1"
                        ctx.blind_start_x = state.x
                        ctx.blind_start_y = state.y
                        return
                    else:
                        print(f"[AUTOPILOT] Triple Gate 2 hilang dari jauh (Area: {ctx.last_front_area}). Fallback Hover & Yaw!")
            
            # Hover in place while rotating to find the gate
            mem_yaw = getattr(ctx, 'last_front_err_x', 0) * ctx.kp_yaw
            mem_yaw = clamp(mem_yaw, -15.0, 15.0)
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)

class DeadReckoning_TripleGate1(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        ctx.dist_flown = math.sqrt((state.x - ctx.blind_start_x)**2 + (state.y - ctx.blind_start_y)**2)
        if ctx.dist_flown < 7.0: # PUNCH THROUGH TUNNEL
            strafe_cmd = 0.0
            if state.lidar_left < 4.9 or state.lidar_right < 4.9:
                strafe_cmd = (state.lidar_right - state.lidar_left) * 0.05
            z_err = -0.8 - state.z
            up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
            
            print(f"[AUTOPILOT] [TRIPLE GATE 2] Blind Punch INS! Jarak: {ctx.dist_flown:.2f}/7.0m, Lidar: {strafe_cmd:.2f}")
            await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
        else:
            print("[AUTOPILOT] Lolos Triple Gate 2! Mencari WP2 (Aruco 2)...")
            ctx.state_phase = "FIND_ARUCO_2"
            ctx.has_seen_target = False


    # ---------------------------------------------------------
    # PHASE 6: FIND_DROPBOX (Mencari Red Drop Box)
    # ---------------------------------------------------------

class AcquireTarget_DropBox(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if down_status == "LOCKED" and down_class in ["Red Drop Box", "RedDrop Box", "Aruco", "Aruco Area"]:
            print("[AUTOPILOT] Red Drop Box / Aruco Terlihat di Kamera Bawah! AUTO-STOP & Memulai Centering...")
            ctx.state_phase = "CENTER_DROPBOX"
            ctx.timeout_counter = 0
            ctx.has_seen_target = False
        else:
            # P-Controller untuk CLIMB ke -1.5 meter secara instan setelah keluar lorong
            z_err = -1.5 - state.z
            climb_cmd = clamp(z_err * 0.8, -0.8, 0.5) # Agresif naik

            # Rule 4: Flat Object Camera Handoff
            # Kalo drop box / aruco kelihatan di kamera depan, steer ke sana
            if front_status == "LOCKED" and front_class in ["Red Drop Box", "RedDrop Box", "Aruco", "Aruco Area"]:
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = True
                ctx.timeout_counter = 0
                ctx.last_front_err_x = front_err_x
                yaw_cmd = front_err_x * ctx.kp_yaw
                
                if abs(front_err_x) > 40:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
            elif ctx.has_seen_target:
                # FALLBACK MEMORY: Masuk blind spot antara kamera depan dan bawah. Creep pelan pakai memori yaw.
                ctx.dist_flown = math.sqrt((state.x - ctx.blind_start_x)**2 + (state.y - ctx.blind_start_y)**2)
                ctx.timeout_counter += 1
                
                if ctx.dist_flown > 2.5 and ctx.timeout_counter < 500:
                    ctx.timeout_counter = 500 # Latch reverse state
                    ctx.reverse_start_x = state.x
                    ctx.reverse_start_y = state.y
                    
                if ctx.timeout_counter >= 500:
                    reverse_dist = math.sqrt((state.x - getattr(ctx, 'reverse_start_x', state.x))**2 + (state.y - getattr(ctx, 'reverse_start_y', state.y))**2)
                    if reverse_dist > 2.0:
                        print("[AUTOPILOT] Mundur kejauhan, Hover nunggu instruksi!")
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        print(f"[AUTOPILOT] Kebablasan Drop Box di blind spot! Terbang mundur...")
                        await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                else:
                    # Stutter creep to level pitch and scan straight down
                    fwd_creep = 0.3 if ctx.timeout_counter % 20 < 10 else 0.0
                    await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
            else:
                # Belum keliatan, jalan lurus pelan sambil nanjak ke ketinggian operasi
                await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                
    # ---------------------------------------------------------
    # PHASE 6.5: CENTER_DROPBOX (Mensejajarkan Drone dengan Drop Box)
    # ---------------------------------------------------------

class TerminalGuidance_DropBox(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if down_status == "LOCKED" and down_class in ["Red Drop Box", "RedDrop Box", "Aruco Area"]:
            ctx.has_seen_target = True
            ctx.last_down_err_x = down_err_x
            ctx.last_down_err_y = down_err_y
            ctx.last_seen_x = state.x
            ctx.last_seen_y = state.y
            
            # Active Auto-Stop Braking (Gentle for Gimbal-less)
            fwd_cmd = -down_err_y * 0.0015
            strafe_cmd = down_err_x * 0.0015
            
            # Limit speed so it brakes instead of overshooting
            fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
            strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
            
            # Z dijaga ketat di -1.5
            z_err = -1.5 - state.z
            up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
            
            print(f"[AUTOPILOT] [DROP BOX] Centering (X:{down_err_x}, Y:{down_err_y}). Fwd:{fwd_cmd:.2f}, Strafe:{strafe_cmd:.2f}, Z:{up_cmd:.2f}")
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            
            if abs(down_err_x) < 20 and abs(down_err_y) < 20:
                if down_class in ["Red Drop Box", "RedDrop Box", "Aruco", "Aruco Area"]:
                    ctx.timeout_counter += 1
                    if ctx.timeout_counter > 100:
                        print("[AUTOPILOT] Medkit Dropped. Yaw Kanan nyari Triple Gate 1...")
                        ctx.state_phase = "YAW_LEFT_TRIPLE_1"
                        ctx.timeout_counter = 0
                        ctx.has_seen_target = False
                else:
                    ctx.timeout_counter = 0
            else:
                ctx.timeout_counter = 0
        else:
            ctx.timeout_counter += 1
            if ctx.timeout_counter > 50:
                print("[AUTOPILOT] Drop Box Hilang dari kamera bawah! Kembali ke FIND_DROPBOX...")
                ctx.state_phase = "FIND_DROPBOX"
                ctx.timeout_counter = 0
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = False # Bener-bener ilang, riset state!
            elif ctx.has_seen_target:
                # FALLBACK MEMORY DOWN CAMERA: Terbang balik ke kordinat GPS terakhir!
                dx = getattr(ctx, 'last_seen_x', state.x) - state.x
                dy = getattr(ctx, 'last_seen_y', state.y) - state.y
                
                yaw_rad = math.radians(state.yaw)
                cos_y = math.cos(yaw_rad)
                sin_y = math.sin(yaw_rad)
                
                # Convert global error to body frame velocity
                fwd_err = dx * cos_y + dy * sin_y
                right_err = -dx * sin_y + dy * cos_y
                
                fwd_cmd = clamp(fwd_err * 0.5, -0.2, 0.2)
                strafe_cmd = clamp(right_err * 0.5, -0.2, 0.2)
                z_err = -1.5 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                print(f"[AUTOPILOT] Drop Box Flicker! Balik ke kordinat memori... Fwd: {fwd_cmd:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0)

    # ---------------------------------------------------------
    # PHASE 7: YAW_RIGHT_TRIPLE_2 (Yaw Kanan ke Triple Gate 1)
    # ---------------------------------------------------------

class ExecuteYawSweep_Right(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        # Simpan YAW awal pas mulai fase ini
        if not hasattr(ctx, 'start_yaw'):
            ctx.start_yaw = state.yaw

        # Hitung selisih muter (Gyroscopic Yaw)
        yaw_diff = abs(state.yaw - ctx.start_yaw)
        if yaw_diff > 180:
            yaw_diff = 360 - yaw_diff

        ctx.timeout_counter += 1
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=30.0)
        
        # HACK: Jangan lock gawang pertama (misi3) yang ada di -108 derajat. 
        # Tunggu muter minimal 120 derajat baru boleh lock gawang misi2!
        if front_status == "LOCKED" and front_class == "Tripple Gate" and front_area > 10000 and yaw_diff > 120:
            print(f"[AUTOPILOT] Triple Gate 1 (Double Gate) terlihat (Area: {front_area}, YawDiff: {yaw_diff:.1f})! Memulai approach...")
            ctx.state_phase = "TRIPLE_GATE_2"
            ctx.timeout_counter = 0
            del ctx.start_yaw

    # ---------------------------------------------------------
    # PHASE 8: TRIPLE_GATE_2 (Lewati lorong ke-2 menuju WP4)
    # ---------------------------------------------------------

class AcquireTarget_TripleGate2(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if front_status == "LOCKED" and front_class == "Tripple Gate":
            ctx.has_seen_target = True
            ctx.last_front_area = front_area
            ctx.last_front_err_y = front_err_y
            ctx.last_front_err_x = front_err_x
            yaw_cmd = front_err_x * ctx.kp_yaw
            
            if front_area < 350000:
                if abs(front_err_x) > 100:
                    fwd_cmd = 0.0 # Rem dulu biar ngadep lurus ke lubang!
                else:
                    fwd_cmd = 0.8
                z_err = -0.8 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
            else:
                yaw_cmd = 0.0 # ANTI-DRIFT: Jangan belok-belok di moncong lorong
                fwd_cmd = 0.8 # FORCE MAJU, jangan ngerem
                z_err = -0.8 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                    
            ctx.timeout_counter = 0
            print(f"[AUTOPILOT] [TRIPLE GATE 1] Centering (Area: {front_area}). Fwd: {fwd_cmd}, Yaw: {yaw_cmd:.2f}, Z: {up_cmd:.2f}")
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=0.0, down_m_s=up_cmd, yaw_deg_s=yaw_cmd)
        else:
            if ctx.has_seen_target:
                ctx.timeout_counter += 1
                if ctx.timeout_counter > 5:
                    if ctx.last_front_area > 350000:
                        print("[AUTOPILOT] Memasuki Lorong Triple Gate 1! Berpindah ke PUNCH_TRIPLE_GATE_2")
                        ctx.state_phase = "PUNCH_TRIPLE_GATE_2"
                        ctx.blind_start_x = state.x
                        ctx.blind_start_y = state.y
                        return
                    else:
                        print(f"[AUTOPILOT] Triple Gate 1 hilang dari jauh (Area: {ctx.last_front_area}). Fallback Hover & Yaw!")
            
            # Hover in place while rotating to find the gate
            mem_yaw = getattr(ctx, 'last_front_err_x', 0) * ctx.kp_yaw
            mem_yaw = clamp(mem_yaw, -15.0, 15.0)
            await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=mem_yaw)

class DeadReckoning_TripleGate2(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        ctx.dist_flown = math.sqrt((state.x - ctx.blind_start_x)**2 + (state.y - ctx.blind_start_y)**2)
        if ctx.dist_flown < 7.0:
            strafe_cmd = 0.0
            if state.lidar_left < 4.9 or state.lidar_right < 4.9:
                strafe_cmd = (state.lidar_right - state.lidar_left) * 0.05
            z_err = -0.8 - state.z
            up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
            
            print(f"[AUTOPILOT] [TRIPLE GATE 1] Blind Punch INS! Jarak: {ctx.dist_flown:.2f}/7.0m, Lidar: {strafe_cmd:.2f}")
            await flight.send_body_velocity(drone, forward_m_s=0.8, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
        else:
            print("[AUTOPILOT] Lolos Triple Gate 1! Mencari Red Drop Box (dan Aruco)...")
            ctx.state_phase = "FIND_DROPBOX"
            ctx.has_seen_target = False


    # ---------------------------------------------------------
    # PHASE 9: FIND_ARUCO_3 (Mencari belokan kiri)
    # ---------------------------------------------------------

class AcquireTarget_Aruco3(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
            print("[AUTOPILOT] Aruco 3 Terlihat di Kamera Bawah! AUTO-STOP & Memulai Centering...")
            ctx.state_phase = "CENTER_ARUCO_3"
            ctx.timeout_counter = 0
            ctx.has_seen_target = False
        else:
            z_err = -1.5 - state.z
            climb_cmd = clamp(z_err * 0.8, -0.8, 0.5)
            
            if front_status == "LOCKED" and front_class in ["Aruco", "Aruco Area"]:
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = True
                ctx.timeout_counter = 0
                yaw_cmd = front_err_x * ctx.kp_yaw
                
                if abs(front_err_x) > 40:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
            elif ctx.has_seen_target:
                ctx.dist_flown = math.sqrt((state.x - ctx.blind_start_x)**2 + (state.y - ctx.blind_start_y)**2)
                ctx.timeout_counter += 1
                
                if ctx.dist_flown > 2.5 and ctx.timeout_counter < 500:
                    ctx.timeout_counter = 500
                    ctx.reverse_start_x = state.x
                    ctx.reverse_start_y = state.y
                    
                if ctx.timeout_counter >= 500:
                    reverse_dist = math.sqrt((state.x - getattr(ctx, 'reverse_start_x', state.x))**2 + (state.y - getattr(ctx, 'reverse_start_y', state.y))**2)
                    if reverse_dist > 2.0:
                        print("[AUTOPILOT] Mundur kejauhan WP3, Hover nunggu instruksi!")
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        print(f"[AUTOPILOT] Kebablasan WP3 di blind spot! Terbang mundur...")
                        await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                else:
                    fwd_creep = 0.3 if ctx.timeout_counter % 20 < 10 else 0.0
                    await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)

    # ---------------------------------------------------------
    # PHASE 9.5: CENTER_ARUCO_3 (Mensejajarkan Drone dengan WP3)
    # ---------------------------------------------------------

class TerminalGuidance_Aruco3(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if down_status == "LOCKED" and down_class in ["Aruco", "Aruco Area"]:
            ctx.has_seen_target = True
            ctx.last_down_err_x = down_err_x
            ctx.last_down_err_y = down_err_y
            
            fwd_cmd = -down_err_y * 0.0015
            strafe_cmd = down_err_x * 0.0015
            fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
            strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
            z_err = -1.5 - state.z
            up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
            
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)
            
            if abs(down_err_x) < 20 and abs(down_err_y) < 20:
                ctx.timeout_counter += 1
                completion_threshold = 50 if down_class == "Aruco" else 150
                if ctx.timeout_counter > completion_threshold:
                    print("[AUTOPILOT] Presisi WP3 Tercapai! Lurus nyari Final Gate 1...")
                    ctx.state_phase = "CENTERING_FINAL_GATE"
                    ctx.timeout_counter = 0
                    ctx.has_seen_target = False
            else:
                ctx.timeout_counter = 0
        else:
            ctx.timeout_counter += 1
            if ctx.timeout_counter > 50:
                print("[AUTOPILOT] WP3 Hilang! Kembali ke FIND_ARUCO_3...")
                ctx.state_phase = "FIND_ARUCO_3"
                ctx.timeout_counter = 0
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = False
            elif ctx.has_seen_target:
                fwd_cmd = clamp(-ctx.last_down_err_y * 0.0015, -0.2, 0.2)
                strafe_cmd = clamp(ctx.last_down_err_x * 0.0015, -0.2, 0.2)
                z_err = -1.5 - state.z
                up_cmd = clamp(z_err * 0.5, -0.5, 0.5)
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=up_cmd, yaw_deg_s=0.0)

    # ---------------------------------------------------------
    # PHASE 10: TURN_ARUCO_3 (Belok Kiri 90 derajat)
    # ---------------------------------------------------------

class ExecuteYawSweep_Aruco3(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        ctx.timeout_counter += 1
        # Belok kiri (yaw_deg_s negatif)
        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=-30.0)
        
        # Kunci Matematis: Area filter > 12000 to ensure we lock the CLOSEST gate (Final Gate 1), not Final Gate 2 in the background.
        if front_status == "LOCKED" and front_class == "Single Gate" and front_area > 12000 and front_err_x < 0:
            print(f"[AUTOPILOT] Final Gate 1 Terkunci di Kiri (Area: {front_area})! Meluncur maju...")
            ctx.state_phase = "CENTERING_FINAL_GATE"
            ctx.timeout_counter = 0
        
        if ctx.timeout_counter > 120: # ~5 detik (Maksimum turning limit)
            print("[AUTOPILOT] Timeout belok! Paksa masuk mode Centering Final Gate...")
            ctx.state_phase = "CENTERING_FINAL_GATE"
            ctx.timeout_counter = 0

    # ---------------------------------------------------------
    # PHASE 11: FIND_FINAL_GATE_1 (Mencari Single Gate pertama setelah WP3)
    # ---------------------------------------------------------

class AcquireTarget_LandingPad(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if down_status == "LOCKED" and down_class in ["Landing path", "Aruco"]:
            print("[AUTOPILOT] Landing Pad Terlihat di Kamera Bawah! AUTO-STOP & Memulai Centering...")
            ctx.state_phase = "PRECISION_LANDING"
            ctx.timeout_counter = 0
            ctx.has_seen_target = False
        else:
            # P-Controller CLIMB ke -1.5m setelah punch through Final Gate 2
            z_err = -1.5 - state.z
            climb_cmd = clamp(z_err * 0.8, -0.8, 0.5)
            
            # Rule 4: Handoff Kamera
            if front_status == "LOCKED" and front_class in ["Landing path", "Aruco"]:
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = True
                ctx.timeout_counter = 0
                ctx.last_front_err_x = front_err_x
                yaw_cmd = front_err_x * ctx.kp_yaw
                
                if abs(front_err_x) > 40:
                    await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
                else:
                    await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=yaw_cmd)
            elif ctx.has_seen_target:
                # FALLBACK MEMORY: Blind spot creep
                ctx.dist_flown = math.sqrt((state.x - ctx.blind_start_x)**2 + (state.y - ctx.blind_start_y)**2)
                ctx.timeout_counter += 1
                
                if ctx.dist_flown > 2.5 and ctx.timeout_counter < 500:
                    ctx.timeout_counter = 500 # Latch reverse state
                    ctx.reverse_start_x = state.x
                    ctx.reverse_start_y = state.y
                    
                if ctx.timeout_counter >= 500:
                    reverse_dist = math.sqrt((state.x - getattr(ctx, 'reverse_start_x', state.x))**2 + (state.y - getattr(ctx, 'reverse_start_y', state.y))**2)
                    if reverse_dist > 2.0:
                        print("[AUTOPILOT] Mundur kejauhan Landing Pad, Hover nunggu instruksi!")
                        await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                    else:
                        print(f"[AUTOPILOT] Kebablasan Landing Pad di blind spot! Terbang mundur...")
                        await flight.send_body_velocity(drone, forward_m_s=-0.3, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
                else:
                    # Stutter creep to level pitch and scan straight down
                    fwd_creep = 0.3 if ctx.timeout_counter % 20 < 10 else 0.0
                    await flight.send_body_velocity(drone, forward_m_s=fwd_creep, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.6, right_m_s=0.0, down_m_s=climb_cmd, yaw_deg_s=0.0)

    # ---------------------------------------------------------
    # PHASE 10: PRECISION_LANDING (Turun pelan sambil centering)
    # ---------------------------------------------------------

class TerminalDescent_Landing(BaseState):
    async def execute(self, drone, ctx):
        front_status = state.target_front.get("status", "LOST")
        front_class = state.target_front.get("class", "none")
        front_err_x = state.target_front.get("error_x", 0)
        front_err_y = state.target_front.get("error_y", 0)

        front_area = state.target_front.get("area", 0)
        front_confident = state.target_front.get("confident", state.target_front.get("confidence", 0.0))

        down_status = state.target_down.get("status", "LOST")
        down_confident = state.target_down.get("confident", state.target_down.get("confidence", 0.0))
        down_class = state.target_down.get("class", "none")
        down_err_x = state.target_down.get("error_x", 0)
        down_err_y = state.target_down.get("error_y", 0)

        # Active Altitude Hold for 1.5m Operating Height
        z_err_15 = -1.5 - state.z
        global_climb_cmd = clamp(z_err_15 * 0.5, -0.5, 0.5)

        # ---------------------------------------------------------
        # PHASE 2: CENTERING_GATE_1 (Maju nembus gawang pertama)
        # ---------------------------------------------------------
        if down_status == "LOCKED" and (down_class == "Landing path" or down_class == "Aruco"):
            ctx.has_seen_target = True
            ctx.last_down_err_x = down_err_x
            ctx.last_down_err_y = down_err_y
            
            # Pake 0.0015 buat Active Braking yang gentle
            fwd_cmd = -down_err_y * 0.0015
            strafe_cmd = down_err_x * 0.0015
            
            # Limit speed so it brakes gently instead of overshooting
            fwd_cmd = clamp(fwd_cmd, -0.2, 0.2)
            strafe_cmd = clamp(strafe_cmd, -0.2, 0.2)
            
            print(f"[AUTOPILOT] [LANDING] Fwd: {fwd_cmd:.2f}, Strafe: {strafe_cmd:.2f}, Stable: {ctx.landing_ticks}/30, Z: {state.z:.2f}")
            # Turun pelan-pelan (0.3 m/s) sambil centering
            await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.3, yaw_deg_s=0.0)
            
            if abs(down_err_x) < 80 and abs(down_err_y) < 80:
                ctx.landing_ticks += 1
                if ctx.landing_ticks > 30: # Stabil 3 detik nyata (cukup, keburu buta kalau kelamaan)
                    print("[AUTOPILOT] Mendarat sempurna di titik tengah!")
                    await drone.action.land()
                    print("[AUTOPILOT] Menunggu 8 detik buat pendaratan fisik sebelum Auto-Reset...")
                    await asyncio.sleep(8)
                    # import os
                    # os.system("./respawn.sh")
                    ctx.state_phase = "DONE"; return
            else:
                ctx.landing_ticks = 0
                ctx.timeout_counter = 0
        else:
            ctx.timeout_counter += 1
            # FORCE LAND: Kalau udah terlalu rendah, kamera bawah pasti buta. Paksa mendarat!
            if state.z > -0.4 and ctx.has_seen_target:
                print(f"[AUTOPILOT] Ketinggian kritis ({state.z:.2f}m)! Kamera bawah buta. FORCE LANDING!")
                await drone.action.land()
                print("[AUTOPILOT] Menunggu 8 detik buat pendaratan fisik sebelum Auto-Reset...")
                await asyncio.sleep(8)
                # import os
                # os.system("./respawn.sh")
                ctx.state_phase = "DONE"; return
            elif ctx.timeout_counter > 50:
                print("[AUTOPILOT] Landing Pad Hilang! Kembali ke FIND_LANDING_PAD...")
                ctx.state_phase = "FIND_LANDING_PAD"
                ctx.timeout_counter = 0
                ctx.landing_ticks = 0
                ctx.blind_start_x = state.x
                ctx.blind_start_y = state.y
                ctx.has_seen_target = False
            elif ctx.has_seen_target:
                # FALLBACK MEMORY DOWN CAMERA
                fwd_cmd = clamp(-ctx.last_down_err_y * 0.0015, -0.2, 0.2)
                strafe_cmd = clamp(ctx.last_down_err_x * 0.0015, -0.2, 0.2)
                print(f"[AUTOPILOT] Landing Pad Flicker! Terbang balik pake memori... Fwd: {fwd_cmd:.2f}, Z: {state.z:.2f}")
                await flight.send_body_velocity(drone, forward_m_s=fwd_cmd, right_m_s=strafe_cmd, down_m_s=0.3, yaw_deg_s=0.0)
            else:
                await flight.send_body_velocity(drone, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.5, yaw_deg_s=0.0)


STATE_REGISTRY = {
    "BLIND_PUNCH_TAKEOFF": BlindPunch_Takeoff(),
    "CENTERING_GATE_1": TerminalGuidance_Gate1(),
    "CENTERING_GATE_2": TerminalGuidance_Gate2(),
    "CENTERING_FINAL_GATE": TerminalGuidance_FinalGate(),
    "FIND_ARUCO_1": AcquireTarget_Aruco1(),
    "CENTER_ARUCO_1": TerminalGuidance_Aruco1(),
    "FOLLOW_LINE_TO_WP2": FlyByWire_LineFollow(),
    "FIND_ARUCO_2": AcquireTarget_Aruco2(),
    "CENTER_ARUCO_2": TerminalGuidance_Aruco2(),
    "YAW_LEFT_TRIPLE_1": ExecuteYawSweep_Left(),
    "FIND_TRIPLE_GATE_1": AcquireTarget_TripleGate1(),
    "PUNCH_TRIPLE_GATE_1": DeadReckoning_TripleGate1(),
    "FIND_DROPBOX": AcquireTarget_DropBox(),
    "CENTER_DROPBOX": TerminalGuidance_DropBox(),
    "YAW_RIGHT_TRIPLE_2": ExecuteYawSweep_Right(),
    "TRIPLE_GATE_2": AcquireTarget_TripleGate2(),
    "PUNCH_TRIPLE_GATE_2": DeadReckoning_TripleGate2(),
    "FIND_ARUCO_3": AcquireTarget_Aruco3(),
    "CENTER_ARUCO_3": TerminalGuidance_Aruco3(),
    "TURN_ARUCO_3": ExecuteYawSweep_Aruco3(),
    "YAW_RIGHT_FINAL_LINE": ExecuteYawSweep_FinalLine(),
    "FIND_LANDING_PAD": AcquireTarget_LandingPad(),
    "PRECISION_LANDING": TerminalDescent_Landing(),
}
