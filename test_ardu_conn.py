import time
from pymavlink import mavutil
import sys
import config

def test_ardu_connection():
    port = config.FC_CONNECTION
    baud = config.FC_BAUD
    
    print(f"[*] Mencoba koneksi ke ArduPilot via {port} dengan baudrate {baud}...")
    try:
        master = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        print(f"[ERROR] Gagal membuka port {port}. Pastikan kabel bener (TX-RX silang). Error: {e}")
        sys.exit(1)

    print("[*] Port terbuka! Menunggu HEARTBEAT dari ArduPilot (Max 10 detik)...")
    
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=10.0)
    if not msg:
        print("[ERROR] Nggak ada HEARTBEAT dari FC! Cek perkabelan (TX Jetson ke RX FC, RX Jetson ke TX FC) dan baudrate.")
        sys.exit(1)
        
    print(f"[SUCCESS] HEARTBEAT DITERIMA! System ID: {master.target_system}, Component ID: {master.target_component}")
    
    # Request data stream
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1
    )

    print("\n[INFO] Menampilkan Data Telemetri... (Tekan Ctrl+C untuk berhenti)")
    print("---------------------------------------------------------------")
    
    try:
        while True:
            msg = master.recv_match(blocking=True, timeout=1.0)
            if not msg:
                continue
                
            msg_type = msg.get_type()
            
            if msg_type == "HEARTBEAT":
                # Print flight mode
                mode = mavutil.mode_string_v10(msg)
                sys.stdout.write(f"\r[STATUS] Mode: {mode:<10} | ")
            elif msg_type == "RC_CHANNELS":
                # Print RC channels to verify switches
                sys.stdout.write(f"CH5 (Mode): {msg.chan5_raw:<4} | CH6 (AI): {msg.chan6_raw:<4} ")
                sys.stdout.flush()
                
    except KeyboardInterrupt:
        print("\n\n[SUCCESS] Test Koneksi ArduPilot Selesai. Komunikasi TX/RX Normal!")

if __name__ == "__main__":
    test_ardu_connection()
