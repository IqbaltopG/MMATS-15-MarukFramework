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
    Contoh Integrasi Vision AI -> Think Layer -> RC Override.
    Menyelaraskan kamera ke gawang lalu melakukan Blind Punch.
    """
    def __init__(self, autopilot):
        super().__init__(autopilot)
        self.is_blind_punching = False
        self.punch_ticks = 0

    async def execute(self):
        # 1. BLIND PUNCH PHASE (Dead Reckoning)
        if self.is_blind_punching:
            print(f"[STATE] 🚀 BLIND PUNCHING... Tick: {self.punch_ticks}")
            
            # Dorong tuas maju 40%. Kunci kemudi Kiri/Kanan.
            await self.flight.send_rc_override(self.master, forward_cmd=0.4, right_cmd=0.0, yaw_cmd=0.0)
            
            self.punch_ticks += 1
            
            # Asumsi: 40% Pitch = 1 m/s. Butuh tembus 2m lorong + 1m ekstra ruang = 3 detik = 30 Ticks
            if self.punch_ticks > 30: 
                print("[STATE] 🛑 SELESAI PUNCH! NGEREM!")
                self.is_blind_punching = False
                self.punch_ticks = 0
                await self.flight.send_rc_override(self.master, forward_cmd=0.0) # Rem (stik tengah)
                self.autopilot.state_phase = "WP2" # Lanjut misi berikutnya
            
            return # Keluar fungsi agar tidak membaca YOLO saat lagi punch buta

        # 2. VISION CENTERING PHASE (Sense -> Think -> Act)
        if global_state.target_locked:
            print(f"[STATE] 🎯 Target {global_state.target_class} ditemukan! Area: {global_state.area}")
            
            # P-Controller: Ubah Pixel Error YOLO menjadi persentase gerakan tuas RC (-1.0 s/d 1.0)
            # Asumsi error maksimal layar adalah 320 piksel. 
            # Kp (Proportional Gain) misalnya 0.002. Jadi error 100 piksel = dorong stik 20% (0.2).
            yaw_cmd = global_state.error_x * 0.002 
            
            # Clamp kemudi maksimal 30% biar nggak kepleset
            yaw_cmd = max(-0.3, min(0.3, yaw_cmd)) 
            
            # THE TRIPWIRE (Kalkulasi Luas Gawang)
            # Jika Area gawang > 120,000 (Gawang udah di depan hidung), DAN hidung udah lurus
            if global_state.area > 120000 and abs(global_state.error_x) < 30:
                print("[STATE] 🚨 TRIPWIRE TERSENTUH! MEMULAI BLIND PUNCH!")
                self.is_blind_punching = True
                return

            # Kalau belum menyentuh Tripwire, perlahan maju sambil terus luruskan hidung (Centering)
            await self.flight.send_rc_override(
                self.master, 
                forward_cmd=0.2, # Maju santai 20%
                right_cmd=0.0, 
                up_cmd=0.0, 
                yaw_cmd=yaw_cmd  # Belok kiri/kanan sesuai posisi YOLO
            )
            
        else:
            print("[STATE] ❓ Target hilang. Sweep Radar (Memory Buffer)...")
            
            # Implementasi "Anti-Tawaf" (Low-Pass Filter) dilakukan di vision_daemon.py
            # Jika masuk ke sini, artinya YOLO sudah buta lebih dari 5 frame.
            # Lakukan sweep mencari target (Putar Yaw Kiri Kanan santai 15%)
            sweep_yaw = math.sin(self.autopilot.timeout_counter * 0.1) * 0.15
            
            # Jangan maju kalau buta, cukup diam di tempat sambil nengok
            await self.flight.send_rc_override(
                self.master, 
                forward_cmd=0.0, 
                right_cmd=0.0, 
                up_cmd=0.0, 
                yaw_cmd=sweep_yaw
            )
