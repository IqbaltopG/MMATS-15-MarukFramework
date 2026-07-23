import time
from pymavlink import mavutil
import sys

def test_connection(port='/dev/ttyTHS1', baud=921600):
    print(f"[*] Mencoba koneksi ke FC via {port} dengan baudrate {baud}...")
    try:
        # Hubungkan ke serial port
        master = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        print(f"[ERROR] Gagal membuka port {port}. Pastikan kabel terpasang dan permission /dev/ttyTHS1 sudah diset (sudo chmod 666 {port}). Error: {e}")
        sys.exit(1)

    print("[*] Port terbuka! Menunggu HEARTBEAT dari DekeFPV (ArduPilot)...")
    
    # Tunggu heartbeat maksimal 10 detik
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=10.0)
    if not msg:
        print("[ERROR] Tidak ada HEARTBEAT dari FC! Cek perkabelan (TX ke RX, RX ke TX) dan pastikan FC menyala.")
        sys.exit(1)
        
    print(f"[SUCCESS] HEARTBEAT DITERIMA! System ID: {master.target_system}, Component ID: {master.target_component}")
    print(f"[INFO] Autopilot: {msg.autopilot}, Type: {msg.type}")

    print("\n[*] Menunggu data Telemetri Dasar (ATTITUDE / Baterai)...")
    
    # Minta stream data (jika belum stream)
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1
    )

    start_time = time.time()
    while time.time() - start_time < 5.0:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg:
            if msg.get_type() == "ATTITUDE":
                print(f"[TELEMETRI] Roll: {msg.roll:.2f} | Pitch: {msg.pitch:.2f} | Yaw: {msg.yaw:.2f}")
            elif msg.get_type() == "SYS_STATUS":
                volt = msg.voltage_battery / 1000.0
                print(f"[BATERAI] Tegangan: {volt:.2f} V")
            elif msg.get_type() == "RC_CHANNELS":
                print(f"[RC INPUT] CH1: {msg.chan1_raw} | CH5 (Mode): {msg.chan5_raw}")

    print("\n[SUCCESS] Test Koneksi Serial selesai. Komunikasi TX/RX Normal!")

if __name__ == "__main__":
    test_connection()
