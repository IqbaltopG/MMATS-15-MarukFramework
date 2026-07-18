import asyncio
import json
import socket
import config
from pymavlink import mavutil
from comms import connect_drone
from states import State_Takeoff, State_SearchTarget

# =====================================================================
# MARUK FRAMEWORK: AUTOPILOT DAEMON (CONTROLLER LAYER)
# =====================================================================

class Autopilot:
    def __init__(self):
        self.master = None
        self.state_phase = "IDLE"
        self.global_vision_state = {}
        self.timeout_counter = 0

    async def setup(self):
        print("[AUTOPILOT] Inisialisasi MARUK Engine...")
        self.master = connect_drone()
        
        self.states = {
            "TAKEOFF": State_Takeoff(self),
            "SEARCH_TARGET": State_SearchTarget(self)
        }

    async def udp_listener(self):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(('127.0.0.1', 5005))
        udp_socket.setblocking(False)
        
        loop = asyncio.get_event_loop()
        print("[AUTOPILOT] UDP Listener berjalan di Port 5005...")
        
        while True:
            try:
                data, _ = await loop.sock_recvfrom(udp_socket, 1024)
                payload = json.loads(data.decode('utf-8'))
                self.global_vision_state.update(payload)
            except Exception:
                pass
            await asyncio.sleep(0.01)

    async def run_state_machine(self):
        print("[AUTOPILOT] State Machine Engine DIMULAI (10Hz)...")
        # Tunggu mode GUIDED masuk
        print("[AUTOPILOT] Pindah ke Mode GUIDED untuk memulai!")
        while True:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                mode = mavutil.mode_string_v10(msg)
                if mode == 'GUIDED':
                    self.state_phase = "SEARCH_TARGET" # Atau Takeoff
                    print("[AUTOPILOT] GUIDED AKTIF! Misi Dimulai!")
                    break
            await asyncio.sleep(0.5)

        while True:
            # Operational Failsafe
            if self.timeout_counter > 300: 
                print("[EMERGENCY] AI BUTA / TIMEOUT! RETURN TO LAUNCH!")
                self.master.set_mode_rtl()
                self.state_phase = "IDLE"
                self.timeout_counter = 0

            # Eksekusi State
            current_state = self.states.get(self.state_phase)
            if current_state:
                await current_state.execute()
                self.timeout_counter += 1
                    
            await asyncio.sleep(0.1)

async def main():
    agent = Autopilot()
    await agent.setup()
    await asyncio.gather(
        agent.udp_listener(),
        agent.run_state_machine()
    )

if __name__ == "__main__":
    asyncio.run(main())
