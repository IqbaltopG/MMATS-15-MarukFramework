import math
import asyncio
from pymavlink import mavutil

import config

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

async def send_body_velocity(drone, forward_m_s: float = 0.0, right_m_s: float = 0.0, down_m_s: float = 0.0, yaw_deg_s: float = 0.0):
    """
    [JALUR SULTAN - ARDUPILOT GUIDED MODE]
    Mengirim perintah kecepatan (Velocity) langsung ke ArduPilot dalam satuan m/s.
    Drone WAJIB berada dalam mode GUIDED!
    """
    master = drone
    
    # Konversi Yaw Rate dari derajat/detik ke radian/detik
    yaw_rate_rad = math.radians(yaw_deg_s)
    
    # Bitmask: Kita mau pakai Velocity X, Y, Z dan Yaw Rate.
    # Sisanya (Position, Acceleration, Yaw Angle) di-IGNORE.
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )
    
    if config.IS_SIMULATION:
        # Hack untuk SITL kalau perlu, tapi ArduPilot SITL dukung penuh NED
        # Kita pakai perintah asli aja biar Simulation sama dengan Real Life
        pass

    # Kirim MAVLink Message 84 (SET_POSITION_TARGET_LOCAL_NED)
    # Gunakan MAV_FRAME_BODY_NED agar maju/mundur mengacu pada moncong drone
    master.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0, # x, y, z positions (di-ignore)
        forward_m_s, right_m_s, down_m_s, # x, y, z velocity dalam m/s
        0, 0, 0, # x, y, z acceleration (di-ignore)
        0, yaw_rate_rad # yaw angle (di-ignore), yaw rate (rad/s)
    )

async def arm_and_takeoff(drone, altitude_m=1.5):
    """
    Takeoff Manual dibantu AI: Pilot Takeoff ke ketinggian 1.5m,
    Lalu Pilot pindah ke mode GUIDED, dan cetek CH5!
    """
    print("[FLIGHT] TAKEOFF DIBYPASS (HARDWARE AGNOSTIC)!")
    print("[FLIGHT] Harap Pilot Takeoff Manual ke 1.5m dan Pindah ke GUIDED.")
    print("[FLIGHT] Lalu cetek Switch CH5 untuk memberikan kendali ke AI!")
    await asyncio.sleep(2)
    
async def hover(drone):
    """
    Berhenti di udara (Netral RC)
    """
    await send_body_velocity(drone, 0.0, 0.0, 0.0, 0.0)

async def release_rc_override(master):
    """Membebaskan RC ke Pilot (Gunakan 65535)"""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
    )
