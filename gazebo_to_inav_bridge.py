import socket
import struct
import time
import math

# =====================================================================
# THE INTERPRETER: GAZEBO HARMONIC -> INAV SITL (X-PLANE PROTOCOL)
# =====================================================================

class XPlaneBridge:
    def __init__(self, inav_ip="127.0.0.1", inav_port=49000):
        self.inav_ip = inav_ip
        self.inav_port = inav_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def pack_xplane_data(self, index, data_list):
        """
        X-Plane UDP protocol expects:
        - 4 bytes int (Index/Message Type)
        - 8 floats (32 bytes data)
        Total 36 bytes per chunk.
        """
        # Ensure we have exactly 8 floats
        while len(data_list) < 8:
            data_list.append(0.0)
        data_list = data_list[:8]
        
        # Pack index as little-endian int, followed by 8 floats
        return struct.pack("<I8f", index, *data_list)

    def send_to_inav(self, pitch, roll, yaw, lat, lon, alt):
        """
        Contoh pengiriman data Attitude (Pitch/Roll/Yaw) dan Posisi (Lat/Lon/Alt).
        Index X-Plane:
        17 = Pitch, Roll, Yaw
        20 = Lat, Lon, Altitude
        """
        # Header X-Plane wajib: "DATA" + 1 null byte
        msg = b"DATA\x00" 
        
        # Data 1: Attitude (Pitch, Roll, Yaw) - Index 17
        # X-Plane format: Pitch, Roll, True Heading, Mag Heading
        attitude_data = self.pack_xplane_data(17, [pitch, roll, yaw, yaw])
        
        # Data 2: Position (Lat, Lon, Alt) - Index 20
        # X-Plane format: Lat, Lon, Alt(MSL)
        position_data = self.pack_xplane_data(20, [lat, lon, alt])
        
        msg += attitude_data + position_data
        
        # Tembak UDP ke iNAV SITL
        self.sock.sendto(msg, (self.inav_ip, self.inav_port))
        # print(f"[BRIDGE] Dikirim ke iNAV: Pitch={pitch:.2f} Roll={roll:.2f} Alt={alt:.2f}")

# ==========================================================
# 2. GAZEBO SUBSCRIBER (Kuping Kiri)
# ==========================================================
# UNCOMMENT JIKA gz-transport12 SUDAH DIINSTALL DI JETSON LU!
"""
from gz.transport12 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

def gazebo_pose_callback(msg):
    for pose in msg.pose:
        if pose.name == "drone": # Ganti sesuai nama model drone lu di SDF
            # Extract Quaternion -> Pitch, Roll, Yaw (konversi quaternion to euler)
            q = pose.orientation
            pitch = math.asin(2.0 * (q.w * q.y - q.z * q.x))
            roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            
            # Extract Position -> Lat, Lon, Alt (Offset dari Origin Gazebo ke titik GPS)
            lat = -7.250445 + (pose.position.x * 0.00001) # Konversi kasar Meter ke Derajat
            lon = 112.768845 + (pose.position.y * 0.00001)
            alt = pose.position.z
            
            bridge.send_to_inav(pitch, roll, 0.0, lat, lon, alt)

def start_gazebo_subscriber():
    node = Node()
    topic = "/model/drone/pose"
    node.subscribe(Pose_V, topic, gazebo_pose_callback)
    print(f"[BRIDGE] Menunggu data sensor Gazebo di topik: {topic}")
"""

if __name__ == "__main__":
    bridge = XPlaneBridge()
    print("[BRIDGE] Interpreter Gazebo -> iNAV SITL (X-Plane) Siap!")
    print("[BRIDGE] Sedang mengirim data dummy (Heartbeat)...")
    
    # DUMMY LOOP: Simulasi dapet data dari Gazebo
    t = 0
    try:
        while True:
            # Simulasi drone goyang dikit (Sine wave)
            fake_pitch = math.sin(t) * 5.0 
            fake_roll = math.cos(t) * 5.0
            fake_alt = 10.0 + math.sin(t) * 2.0
            
            bridge.send_to_inav(
                pitch=fake_pitch, 
                roll=fake_roll, 
                yaw=0.0, 
                lat=-7.250445, # Dummy Surabaya
                lon=112.768845, 
                alt=fake_alt
            )
            t += 0.1
            time.sleep(0.1) # 10Hz
    except KeyboardInterrupt:
        print("\n[BRIDGE] Dimatikan.")
