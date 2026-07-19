import asyncio
from pymavlink import mavutil
from comms import connect_drone, global_state, mavlink_router_task, start_udp_server
from states import State_Takeoff, State_SearchTarget

# =====================================================================
# MARUK FRAMEWORK: AUTOPILOT DAEMON (CONTROLLER LAYER)
# =====================================================================

class Autopilot:
    def __init__(self):
        self.master = None
        self.state_phase = "IDLE"
        self.timeout_counter = 0

    async def setup(self):
        print("[AUTOPILOT] Inisialisasi MARUK Engine...")
        self.master = connect_drone()
        
        self.states = {
            "TAKEOFF": State_Takeoff(self),
            "SEARCH_TARGET": State_SearchTarget(self)
        }

    async def run_state_machine(self):
        print("[AUTOPILOT] State Machine Engine DIMULAI (10Hz)...")
        print("[AUTOPILOT] Pindah ke Mode GUIDED untuk memulai!")
        
        while True:
            # Pengecekan Failsafe Buta
            if self.timeout_counter > 300: 
                print("[EMERGENCY] AI BUTA / TIMEOUT! RETURN TO LAUNCH!")
                self.master.set_mode_rtl()
                self.state_phase = "IDLE"
                self.timeout_counter = 0

            # Gunakan global_state.mode yang dibaca oleh comms.py (CH5 biasanya mengatur ini di ArduPilot)
            if global_state.mode == 'GUIDED':
                
                # =============================================================
                # FITUR RESUME / RESET WAYPOINT DARI KNOB REMOTE (CH7)
                # =============================================================
                if global_state.ch7_knob < 1300 and self.state_phase != "TAKEOFF":
                    if self.state_phase != "WP1":
                        print("[AUTOPILOT] KNOB KIRI: Reset misi ke WP1!")
                        self.state_phase = "WP1"
                        
                elif 1400 < global_state.ch7_knob < 1600 and self.state_phase != "TAKEOFF":
                    if self.state_phase != "WP2":
                        print("[AUTOPILOT] KNOB TENGAH: Reset misi ke WP2!")
                        self.state_phase = "WP2"
                        
                elif global_state.ch7_knob > 1700 and self.state_phase != "TAKEOFF":
                    if self.state_phase != "WP3":
                        print("[AUTOPILOT] KNOB KANAN: Reset misi ke WP3!")
                        self.state_phase = "WP3"
                # =============================================================

                if self.state_phase == "IDLE":
                    self.state_phase = "SEARCH_TARGET"
                    print("[AUTOPILOT] GUIDED AKTIF! Misi Dimulai!")

                current_state = self.states.get(self.state_phase)
                if current_state:
                    await current_state.execute()
                    self.timeout_counter += 1
            else:
                if self.state_phase != "IDLE":
                    print("[AUTOPILOT] Mode bukan GUIDED. AI Pause...")
                    self.state_phase = "IDLE"
                    
            await asyncio.sleep(0.1)

async def main():
    agent = Autopilot()
    await agent.setup()
    
    # Start UDP Server dari comms.py
    udp_transport = await start_udp_server()
    
    try:
        await asyncio.gather(
            mavlink_router_task(agent.master),
            agent.run_state_machine()
        )
    finally:
        udp_transport.close()

if __name__ == "__main__":
    asyncio.run(main())
