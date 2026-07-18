import math
from pymavlink import mavutil

# =====================================================================
# MARUK FRAMEWORK: FLIGHT CONTROLLER ABSTRACTION (PYMAVLINK)
# 100% Hardware Agnostic untuk ArduPilot (DekeFPV H743).
# =====================================================================

class FlightController:
    async def send_body_velocity(self, master, forward_m_s: float = 0.0, right_m_s: float = 0.0, down_m_s: float = 0.0, yaw_deg_s: float = 0.0):
        """
        Mengirimkan perintah kecepatan berdasarkan koordinat body (badan drone).
        Menggunakan MAV_FRAME_BODY_NED.
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
