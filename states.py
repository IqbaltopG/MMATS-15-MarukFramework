import asyncio
import math
import config
from flight import FlightController
from comms import global_state

# =====================================================================
# MARUK FRAMEWORK: STATES (THINK LAYER)
# Template kosong untuk State Machine KRTI 2026.
# =====================================================================

class StateBase:
    def __init__(self, autopilot):
        self.autopilot = autopilot
        self.master = autopilot.master
        self.flight = FlightController()
        
    async def execute(self):
        raise NotImplementedError("Setiap state harus mengimplementasikan fungsi execute()")

class State_Takeoff(StateBase):
    async def execute(self):
        print("[STATE] AI MENUNGGU DI MODE GUIDED...")
        # Di dunia nyata (dan arsitektur MARUK), Takeoff dilakukan MANUAL ke ALT_HOLD.
        # AI hanya aktif saat pilot memindah switch ke GUIDED.
        # Jadi tidak ada MAV_CMD_NAV_TAKEOFF di sini.
        pass

class State_SearchTarget(StateBase):
    """
    Contoh State 2: Mencari target menggunakan data dari Vision Daemon.
    """
    async def execute(self):
        if global_state.target_locked:
            print(f"[STATE] Target {global_state.target_class} ditemukan! Mulai Centering...")
            
            # Hitung Kinematika Proporsional
            yaw_cmd = global_state.error_x * config.KP_YAW
            yaw_cmd = max(-30.0, min(30.0, yaw_cmd)) # Batasi kecepatan putar
            
            await self.flight.send_body_velocity(
                self.master, 
                forward_m_s=config.FWD_MAX_SPEED, 
                right_m_s=0.0, 
                down_m_s=0.0, 
                yaw_deg_s=yaw_cmd
            )
        else:
            print("[STATE] Target hilang. Radar Sweep...")
            # Sweep sinusoidal buat nyari target
            sweep_yaw = math.sin(self.autopilot.timeout_counter * 0.1) * 20.0
            await self.flight.send_body_velocity(self.master, forward_m_s=0.2, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=sweep_yaw)
