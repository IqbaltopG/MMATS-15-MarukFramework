import asyncio
import json
from pymavlink import mavutil
import config

# =====================================================================
# MARUK FRAMEWORK: COMMS LAYER
# Sentralisasi semua komunikasi I/O (Serial MAVLink & UDP Vision)
# =====================================================================

class GlobalState:
    def __init__(self):
        # FC State (Dari MAVLink)
        self.mode = "UNKNOWN"
        self.lidar_left = 0.0
        self.lidar_right = 0.0
        self.altitude = 0.0
        self.ch7_knob = 1500
        
        # Vision State (Dari UDP YOLO)
        self.target_locked = False
        self.error_x = 0.0
        self.error_y = 0.0
        self.area = 0.0
        self.target_class = "None"

global_state = GlobalState()

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
                global_state.mode = mavutil.mode_string_v10(msg)
            elif mtype == 'RC_CHANNELS':
                global_state.ch7_knob = msg.chan7_raw
            elif mtype == 'DISTANCE_SENSOR':
                if msg.id == 1:
                    global_state.lidar_left = msg.current_distance / 100.0
                elif msg.id == 2:
                    global_state.lidar_right = msg.current_distance / 100.0
            elif mtype == 'GLOBAL_POSITION_INT':
                global_state.altitude = msg.relative_alt / 1000.0
                
        # Heartbeat Wajib!
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        await asyncio.sleep(0.01)

# ---------------------------------------------------------
# 2. KONEKSI UDP (VISION DAEMON)
# ---------------------------------------------------------
class UDPReceiverProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        try:
            payload = json.loads(data.decode('utf-8'))
            global_state.target_locked = payload.get("target_locked", False)
            global_state.error_x = payload.get("error_x", 0.0)
            global_state.error_y = payload.get("error_y", 0.0)
            global_state.area = payload.get("area", 0.0)
            global_state.target_class = payload.get("class", "None")
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
