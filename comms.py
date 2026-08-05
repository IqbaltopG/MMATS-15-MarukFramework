import asyncio
import json
import math
from pymavlink import mavutil
import config

# =====================================================================
# MARUK FRAMEWORK: COMMS LAYER (Adapted for PX4/INAV RC_OVERRIDE)
# =====================================================================

class DroneState:
    def __init__(self):
        # Coordinates and Attitude
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        
        # Distance Sensors
        self.lidar_left = 5.0
        self.lidar_right = 5.0
        
        # Vision Payloads
        self.target_front = {"status": "LOST", "class": "none", "error_x": 0, "error_y": 0, "area": 0, "confident": 0.0}
        self.target_down = {"status": "LOST", "class": "none", "error_x": 0, "error_y": 0, "area": 0, "confident": 0.0}
        
        # MMATS Hardware Failsafe / Modes
        self.mode = "UNKNOWN"
        self.ch7_knob = 1500
        self.ch5_switch = 1000

# Exporting exactly what states.py expects
state = DroneState()
global_state = state # Alias untuk file lain jika perlu

# ---------------------------------------------------------
# 1. KONEKSI SERIAL / MAVLINK (FLIGHT CONTROLLER)
# ---------------------------------------------------------
def connect_drone():
    if config.IS_SIMULATION:
        print(f"[COMMS] Menghubungkan ke Simulator via {config.FC_CONNECTION}")
        master = mavutil.mavlink_connection(config.FC_CONNECTION)
    else:
        print(f"[COMMS] Menghubungkan ke Fisik Serial via {config.FC_CONNECTION}")
        master = mavutil.mavlink_connection(config.FC_CONNECTION, baud=config.FC_BAUD)

    master.wait_heartbeat()
    return master

async def mavlink_router_task(master):
    print("[COMMS] MAVLink Router Task Berjalan...")
    while True:
        msg = master.recv_match(blocking=False)
        if msg:
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT':
                state.mode = mavutil.mode_string_v10(msg)
            elif mtype == 'RC_CHANNELS':
                state.ch7_knob = msg.chan7_raw
                state.ch5_switch = msg.chan5_raw
            elif mtype == 'DISTANCE_SENSOR':
                if msg.id == 1:
                    state.lidar_left = msg.current_distance / 100.0
                elif msg.id == 2:
                    state.lidar_right = msg.current_distance / 100.0
            elif mtype == 'LOCAL_POSITION_NED':
                state.x = msg.x
                state.y = msg.y
                state.z = msg.z
            elif mtype == 'ATTITUDE':
                state.yaw = msg.yaw * 57.2958 # convert rad to deg
                
        # Heartbeat Wajib!
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        await asyncio.sleep(0.01)

# ---------------------------------------------------------
# 2. KONEKSI UDP (VISION DAEMON)
# ---------------------------------------------------------
class UDPReceiverProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        try:
            message = data.decode('utf-8')
            parsed = json.loads(message)
            
            if parsed.get("camera") == "lidar":
                side = parsed.get("side")
                if side == "left":
                    state.lidar_left = float(parsed.get("range", 5.0))
                elif side == "right":
                    state.lidar_right = float(parsed.get("range", 5.0))
            elif parsed.get("camera") == "down":
                state.target_down.update(parsed)
            else:
                state.target_front.update(parsed)
        except Exception:
            pass

async def start_udp_server(ip="127.0.0.1", port=5005):
    loop = asyncio.get_running_loop()
    print(f"[COMMS] UDP Vision Receiver Berjalan di Port {port}...")
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPReceiverProtocol(),
        local_addr=(ip, port)
    )
    return transport
