from pymavlink import mavutil
import config

def connect_drone():
    """
    Membangun koneksi MAVLink mentah ke Flight Controller (ArduPilot).
    Otomatis mendeteksi Simulasi (UDP) atau Hardware Fisik (Serial).
    """
    if config.IS_SIMULATION:
        print(f"[COMMS] Menghubungkan ke Simulator via {config.FC_CONNECTION}")
        master = mavutil.mavlink_connection(config.FC_CONNECTION)
    else:
        print(f"[COMMS] Menghubungkan ke Fisik Serial via {config.FC_CONNECTION} Baud: {config.FC_BAUD}")
        master = mavutil.mavlink_connection(config.FC_CONNECTION, baud=config.FC_BAUD)

    print("[COMMS] Menunggu Heartbeat dari wahana...")
    master.wait_heartbeat()
    print(f"[COMMS] Wahana terhubung! Sistem ID: {master.target_system}")
    
    return master
