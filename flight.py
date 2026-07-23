import math
from pymavlink import mavutil

# =====================================================================
# MARUK FRAMEWORK: FLIGHT CONTROLLER ABSTRACTION (PYMAVLINK)
# 100% Hardware Agnostic untuk ArduPilot (DekeFPV H743).
# =====================================================================

class FlightController:
    async def send_body_velocity(self, master, forward_m_s: float = 0.0, right_m_s: float = 0.0, down_m_s: float = 0.0, yaw_deg_s: float = 0.0):
        """
        [DEPRECATED FOR GPS-DENIED]
        Mengirimkan perintah kecepatan berdasarkan koordinat body (badan drone).
        Ini akan DITOLAK oleh ArduPilot jika tidak ada GPS / Optical Flow.
        """
        yaw_rad_s = math.radians(yaw_deg_s)
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, # Target frame
            0x07C7, # type_mask: Ignore Pos, Accel, Yaw. Use Vel, YawRate.
            0, 0, 0, # Position
            forward_m_s, right_m_s, down_m_s, # Velocity
            0, 0, 0, # Accel
            0, yaw_rad_s # Yaw (ignored), Yaw rate
        )

    async def send_rc_override(self, master, forward_cmd: float = 0.0, right_cmd: float = 0.0, up_cmd: float = 0.0, yaw_cmd: float = 0.0):
        """
        [THE SLIPPIN' JIMMY HACK FOR GPS-DENIED WITHOUT OPTICAL FLOW]
        Meniru input joystick manusia. Bisa dipakai di mode ALT_HOLD tanpa butuh GPS/Optical Flow!
        Input berupa nilai rasio -1.0 hingga 1.0.
        """
        # Mapping -1.0 sampai 1.0 ke PWM 1000 - 2000 (Tengah 1500)
        # CH1 = Roll (Kanan/Kiri) -> Kanan = PWM > 1500
        # CH2 = Pitch (Maju/Mundur) -> Maju (Nunduk) = PWM < 1500
        # CH3 = Throttle (Naik/Turun) -> Naik = PWM > 1500
        # CH4 = Yaw (Putar Kiri/Kanan) -> Kanan = PWM > 1500
        
        # Deadband / Limitasi agresivitas (JANGAN SET 500 penuh biar drone ga salto)
        # 200 artinya mentok di PWM 1700 atau 1300. Cukup untuk jalan pelan di indoor.
        max_pwm_delta = 200 
        
        pwm_roll = int(1500 + (right_cmd * max_pwm_delta))
        pwm_pitch = int(1500 - (forward_cmd * max_pwm_delta)) # Dibalik, maju = nunduk = PWM turun
        pwm_throttle = int(1500 + (up_cmd * max_pwm_delta))
        pwm_yaw = int(1500 + (yaw_cmd * max_pwm_delta))

        # Clamp nilai PWM antara 1000 dan 2000 untuk safety
        pwm_roll = max(1000, min(2000, pwm_roll))
        pwm_pitch = max(1000, min(2000, pwm_pitch))
        pwm_throttle = max(1000, min(2000, pwm_throttle))
        pwm_yaw = max(1000, min(2000, pwm_yaw))

        # Kirim RC Override ke 4 channel pertama.
        # Angka 65535 artinya "Kembalikan kendali channel ini ke pilot (Remote Asli)"
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            pwm_roll, pwm_pitch, pwm_throttle, pwm_yaw,
            65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
        )
