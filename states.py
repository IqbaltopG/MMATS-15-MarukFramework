import asyncio
import math
import config
from flight import FlightController

# =====================================================================
# MARUK FRAMEWORK: STATES (THINK LAYER)
# Template kosong untuk State Machine KRTI 2026.
# Silakan kembangkan kelas-kelas ini sesuai misi rintangan nanti.
# =====================================================================

class StateBase:
    def __init__(self, autopilot):
        self.autopilot = autopilot
        self.master = autopilot.master
        self.flight = FlightController()
        
    async def execute(self):
        raise NotImplementedError("Setiap state harus mengimplementasikan fungsi execute()")


class State_Takeoff(StateBase):
    """
    Contoh State 1: Drone Takeoff ke ketinggian tertentu.
    """
    async def execute(self):
        print("[STATE] Mengeksekusi Takeoff...")
        await self.master.action.arm()
        await self.master.action.takeoff()
        await asyncio.sleep(5) # Tunggu sampai stabil
        
        # Transisi ke misi selanjutnya
        self.autopilot.state_phase = "SEARCH_TARGET"


class State_SearchTarget(StateBase):
    """
    Contoh State 2: Mencari target menggunakan data dari Vision Daemon.
    """
    async def execute(self):
        vision_data = self.autopilot.global_vision_state
        
        if vision_data.get("target_locked", False):
            print(f"[STATE] Target {vision_data['class']} ditemukan! Mulai Centering...")
            
            # Hitung Kinematika Proporsional berdasarkan Config
            err_x = vision_data.get("error_x", 0.0)
            yaw_cmd = err_x * config.KP_YAW
            
            # Batasi kecepatan putar
            yaw_cmd = max(-30.0, min(30.0, yaw_cmd))
            
            await self.flight.send_body_velocity(
                self.master, 
                forward_m_s=config.FWD_MAX_SPEED, 
                right_m_s=0.0, 
                down_m_s=0.0, 
                yaw_deg_s=yaw_cmd
            )
        else:
            print("[STATE] Target hilang. Radar Sweep...")
            # Contoh Radar Sweep (sinusoidal)
            sweep_yaw = math.sin(self.autopilot.timeout_counter * 0.1) * 20.0
            await self.flight.send_body_velocity(self.master, forward_m_s=0.2, yaw_deg_s=sweep_yaw)

