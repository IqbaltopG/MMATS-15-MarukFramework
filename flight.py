import math
import asyncio
from pymavlink import mavutil

import config

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

async def send_body_velocity(drone, forward_m_s: float = 0.0, right_m_s: float = 0.0, down_m_s: float = 0.0, yaw_deg_s: float = 0.0):
    """
    [THE SLIPPIN' JIMMY ADAPTER]
    Menerima input m/s dari states.py (format MAVSDK Offboard),
    dan men-translate-nya menjadi Joystick Input.
    """
    master = drone
    
    # Tuning kasar m/s ke skala PWM joystick (-1.0 ke 1.0)
    # Anggap 1 m/s setara 0.5 throw joystick
    forward_cmd = forward_m_s / 2.0  
    right_cmd = right_m_s / 2.0      
    up_cmd = -down_m_s / 1.0        # down_m_s positif = turun. up_cmd positif = naik.
    yaw_cmd = yaw_deg_s / 30.0      
    
    max_pwm_delta = 300 # Maksimal 300 PWM (1800 atau 1200) biar aman dari salto
    
    pwm_roll = int(1500 + (right_cmd * max_pwm_delta))
    pwm_pitch = int(1500 - (forward_cmd * max_pwm_delta)) # Maju (Nunduk) = PWM < 1500
    pwm_throttle = int(1500 + (up_cmd * max_pwm_delta))
    pwm_yaw = int(1500 + (yaw_cmd * max_pwm_delta))

    # Clamp untuk safety (Maksimal input 60%)
    pwm_roll = max(1200, min(1800, pwm_roll))
    pwm_pitch = max(1200, min(1800, pwm_pitch))
    pwm_throttle = max(1200, min(1800, pwm_throttle))
    pwm_yaw = max(1200, min(1800, pwm_yaw))

    if config.IS_SIMULATION:
        # HACK GHAIB: PX4 SITL benci RC_CHANNELS_OVERRIDE (Nolak masuk mode).
        # Jadi kita konversi PWM INAV lu balik ke bahasa Virtual Joystick PX4 (MANUAL_CONTROL).
        # Secara fisika, ini 100% SAMA REALISTISNYA dengan PWM! (Murni ngebajak stik, bukan Velocity Offboard)
        x_pitch = int((1500 - pwm_pitch) / max_pwm_delta * 1000) # PWM 1200 -> 1000 (Full Forward)
        y_roll = int((pwm_roll - 1500) / max_pwm_delta * 1000)   # PWM 1800 -> 1000 (Full Right)
        z_throttle = int((pwm_throttle - 1000) / 1000.0 * 1000)  # PWM 1500 -> 500 (Hover)
        r_yaw = int((pwm_yaw - 1500) / max_pwm_delta * 1000)     # PWM 1800 -> 1000 (Yaw Right)
        
        master.mav.manual_control_send(
            master.target_system,
            x_pitch, y_roll, z_throttle, r_yaw,
            0 # buttons
        )
    else:
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            pwm_roll, pwm_pitch, pwm_throttle, pwm_yaw,
            65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
        )

async def arm_and_takeoff(drone, altitude_m=1.5):
    """
    Override dari MAVSDK Takeoff. INAV tidak bisa takeoff otomatis via RC_OVERRIDE tanpa tuning PID yang pas.
    Biar aman, kita suruh pilot manual takeoff ke hover, baru switch CH5 nyala.
    """
    print("[FLIGHT] TAKEOFF DIBYPASS (HARDWARE AGNOSTIC)!")
    print("[FLIGHT] Harap Pilot Takeoff Manual ke 1.5m dan Pindah ke POSHOLD / LOITER.")
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
