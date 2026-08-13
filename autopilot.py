import asyncio
from pymavlink import mavutil
from comms import connect_drone, state, mavlink_router_task, start_udp_server
from states_krti import STATE_REGISTRY, MissionContext
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
                state.ai_switch = 2000

            # 1. HACK CYBORG SWITCH: BACA RAW PWM SAKLAR AI (CH6)
            if state.ai_switch < 1300:
                if self.ctx.state_phase != "IDLE":
                    print("[AUTOPILOT] AI Paused! Saklar AI di bawah (Kendali Manual).")
                    self.ctx.state_phase = "IDLE"
                    await flight.release_rc_override(self.master)
                
                await asyncio.sleep(0.1)
                continue # Skip logic state, diam di tempat (Pilot Manual)
            
            # 2. JIKA SAKLAR AI > 1700 (AI ACTIVE) TAPI STATE MASIH IDLE (Baru di-resume)
            if self.ctx.state_phase == "IDLE":
                # HACK DFA AMNESIA: TENTUKAN STATE RESUME BERDASARKAN KNOB (CH8)
                pwm_knob = state.memory_knob
                if pwm_knob < 1300:
                    print("[AUTOPILOT] 🤖 FULL AUTO TAKEOFF SEQUENCE INITIATED!")
                    await flight.arm_and_takeoff(self.master, altitude_m=1.0)
                    self.ctx.state_phase = "YAW_RIGHT"
                    print("[AUTOPILOT] 🤖 CYBORG RESUME: KNOB KIRI -> Mulai dari Belok Kanan ke Double Gate!")
                elif 1400 < pwm_knob < 1600:
                    await flight.set_mode_guided(self.master)
                    self.ctx.state_phase = "DROP_MEDKIT"
                    print("[AUTOPILOT] 🤖 CYBORG RESUME: KNOB TENGAH -> Mulai dari Drop Medkit!")
                elif pwm_knob > 1700:
                    await flight.set_mode_guided(self.master)
                    self.ctx.state_phase = "YAW_LEFT"
                    print("[AUTOPILOT] 🤖 CYBORG RESUME: KNOB KANAN -> Mulai dari Belok Kiri ke Triple Gate!")
                else:
                    print("[AUTOPILOT] 🤖 FULL AUTO TAKEOFF SEQUENCE INITIATED (Fallback)!")
                    await flight.arm_and_takeoff(self.master, altitude_m=1.0)
                    self.ctx.state_phase = "YAW_RIGHT" # Fallback aman
                    print("[AUTOPILOT] 🤖 CYBORG RESUME: Default -> Belok Kanan ke Double Gate!")

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
