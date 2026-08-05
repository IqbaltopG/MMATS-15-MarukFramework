import asyncio
from pymavlink import mavutil
from comms import connect_drone, state, mavlink_router_task, start_udp_server
from states import STATE_REGISTRY, MissionContext
from config import IS_SIMULATION
import flight

# =====================================================================
# MARUK FRAMEWORK: AUTOPILOT DAEMON (CONTROLLER LAYER)
# =====================================================================

class Autopilot:
    def __init__(self):
        self.master = None
        self.ctx = MissionContext()

    async def setup(self):
        print("[AUTOPILOT] Inisialisasi MARUK Engine (PX4/INAV RC_OVERRIDE)...")
        self.master = connect_drone()
        
        if IS_SIMULATION:
            print("[AUTOPILOT] Mode Simulasi Aktif! Membajak PX4 secara otomatis...")
            # 1. Set Parameter PX4 biar nerima RC Override & Matiin RC Failsafe
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                b'COM_RC_IN_MODE',
                1, # 1 = Joystick
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            self.master.mav.param_set_send(
                self.master.target_system, self.master.target_component,
                b'NAV_RCL_ACT',
                0, # 0 = Disable RC Loss Failsafe
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            
            # SPAM VIRTUAL JOYSTICK DI BACKGROUND BIAR PX4 GAK NGERASA RC LOST SAAT SETUP!
            self.setup_rc_active = True
            async def spam_rc():
                while self.setup_rc_active:
                    self.master.mav.manual_control_send(
                        self.master.target_system,
                        0, 0, 500, 0, 0 # Pitch 0, Roll 0, Throttle 500 (Hover), Yaw 0
                    )
                    await asyncio.sleep(0.1)
            
            rc_task = asyncio.create_task(spam_rc())

            # 2. Force Arming & Takeoff
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
            )
            await asyncio.sleep(1)
            
            # Kirim Takeoff pake 'NaN' di posisi biar dia nganggap "Takeoff di titik ini sekarang juga"
            NaN = float('nan')
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 
                NaN, NaN, NaN, NaN, NaN, NaN, 1.5
            )
            print("[AUTOPILOT] Menunggu drone mencapai ketinggian 1 meter (Anti-RTF Lambat)...")
            
            # Loop pintar nunggu ketinggian (z bernilai negatif kalau naik di NED frame)
            timeout = 30 # Maksimal nunggu 30 detik *real-world*
            while state.z > -1.0 and timeout > 0:
                await asyncio.sleep(1)
                timeout -= 1
                
            print("[AUTOPILOT] Ketinggian aman tercapai!")
            
            # Berhentiin spammer RC dummy, karena main loop states.py udah mau jalan
            self.setup_rc_active = False
            await rc_task

            print("[AUTOPILOT] Pindah Mode ke Position Control (Murni PWM Simulation)!")
            # 3. Pindah Mode ke Position Control (Mode 3)
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3, 0, 0, 0, 0, 0
            )

    async def run_state_machine(self):
        print("[AUTOPILOT] State Machine Engine DIMULAI (10Hz)...")
        print("[AUTOPILOT] Pindah ke Mode POSHOLD / LOITER untuk memulai!")
        
        while True:
            # BYPASS UNTUK SIMULATOR (Tidak perlu remot fisik)
            if IS_SIMULATION:
                state.ch5_switch = 2000

            # 1. HACK CYBORG SWITCH: BACA RAW PWM CH5 (SAKLAR AI)
            if state.ch5_switch < 1300:
                if self.ctx.state_phase != "IDLE":
                    print("[AUTOPILOT] AI Paused! Saklar CH5 di bawah (Kendali Manual).")
                    self.ctx.state_phase = "IDLE"
                    await flight.release_rc_override(self.master)
                
                # 2. HACK DFA AMNESIA: RESET WAYPOINT DARI KNOB CH7 SAAT PAUSE
                pwm_ch7 = state.ch7_knob
                if pwm_ch7 < 1300 and self.ctx.state_phase != "BLIND_PUNCH_TAKEOFF":
                    print("[AUTOPILOT] 🔄 KNOB KIRI: Memori AI digeser ke BLIND_PUNCH_TAKEOFF!")
                    self.ctx.state_phase = "BLIND_PUNCH_TAKEOFF"
                elif 1400 < pwm_ch7 < 1600 and self.ctx.state_phase != "CENTERING_GATE_1":
                    print("[AUTOPILOT] 🔄 KNOB TENGAH: Memori AI digeser ke CENTERING_GATE_1!")
                    self.ctx.state_phase = "CENTERING_GATE_1"
                elif pwm_ch7 > 1700 and self.ctx.state_phase != "FIND_ARUCO_1":
                    print("[AUTOPILOT] 🔄 KNOB KANAN: Memori AI digeser ke FIND_ARUCO_1!")
                    self.ctx.state_phase = "FIND_ARUCO_1"
                
                await asyncio.sleep(0.1)
                continue # Skip logic state, diam di tempat
            
            # Jika CH5 > 1700 (AI Active / Saklar Atas)
            if self.ctx.state_phase == "IDLE":
                # Fallback to a default if switched on without setting knob
                self.ctx.state_phase = "BLIND_PUNCH_TAKEOFF" 
                print("[AUTOPILOT] 🤖 CYBORG SWITCH AKTIF! AI Mengambil Alih!")

            current_state = STATE_REGISTRY.get(self.ctx.state_phase)
            if current_state:
                # Mengirim master sebagai 'drone' ke states.py
                await current_state.execute(self.master, self.ctx)
                if self.ctx.state_phase == 'DONE':
                    print("[AUTOPILOT] MISSION ACCOMPLISHED! Melepas kendali...")
                    await flight.release_rc_override(self.master)
                    break
            else:
                print(f"[AUTOPILOT] UNKNOWN STATE: {self.ctx.state_phase}")
                await flight.release_rc_override(self.master)
                break
                
            await asyncio.sleep(0.1) # 10Hz Loop

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
