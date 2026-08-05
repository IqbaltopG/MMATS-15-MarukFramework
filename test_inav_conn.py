import time
from pymavlink import mavutil
import sys
import config

def test_inav_connection():
    port = config.FC_CONNECTION
    baud = config.FC_BAUD
    
    print(f"[*] Mencoba koneksi ke INAV FC via {port} dengan baudrate {baud}...")
    try:
        master = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        print(f"[ERROR] Gagal membuka port {port}. Pastikan kabel bener (TX-RX silang) dan permission diset (sudo chmod 666 {port}). Error: {e}")
        sys.exit(1)

    print("[*] Port terbuka! Menunggu HEARTBEAT dari INAV (Max 10 detik)...")
    
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=10.0)
    if not msg:
        print("[ERROR] Nggak ada HEARTBEAT dari FC! Cek perkabelan (TX Jetson ke RX FC, RX Jetson ke TX FC) dan pastikan UART di INAV udah diset ke MAVLink.")
        sys.exit(1)
        
    print(f"[SUCCESS] HEARTBEAT DITERIMA! System ID: {master.target_system}, Component ID: {master.target_component}")
    print(f"[INFO] Autopilot ID: {msg.autopilot} (INAV biasanya emulasi ArduPilot/Generic)")

    print("\n[*] Menunggu data Telemetri Dasar (ATTITUDE / RC INPUT)...")
    
    # Request data stream (INAV butuh ini buat mulai ngirim telemetri secara konstan)
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1
    )

    start_time = time.time()
    seen_att = False
    seen_rc = False
    
    while time.time() - start_time < 5.0:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg:
            if msg.get_type() == "ATTITUDE" and not seen_att:
                print(f"[TELEMETRI] Roll: {msg.roll:.2f} | Pitch: {msg.pitch:.2f} | Yaw: {msg.yaw:.2f}")
                seen_att = True
            elif msg.get_type() == "RC_CHANNELS" and not seen_rc:
                print(f"[RC INPUT] CH1 (Roll): {msg.chan1_raw} | CH5 (Mode Switch): {msg.chan5_raw}")
                seen_rc = True
            elif msg.get_type() == "SYS_STATUS":
                volt = msg.voltage_battery / 1000.0
                if volt > 0:
                    print(f"[BATERAI] Tegangan: {volt:.2f} V")

    print("\n[SUCCESS] Test Koneksi MAVLink INAV Selesai. Komunikasi TX/RX Normal! Gas tempur bos!")

if __name__ == "__main__":
    test_inav_connection()
