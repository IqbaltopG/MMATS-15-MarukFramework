#!/usr/bin/env python3
"""
🛰️ VIO DAEMON (Software-Defined Optical Flow & MAVLink Injection)
==================================================================
Boilerplate "Akal-Akalan Barat" untuk menyuap EKF ArduPilot agar membuka
fitur VELOCITY CONTROL (send_body_velocity) tanpa sensor Optical Flow fisik!

Cara Kerja:
1. Membaca kamera bawah (Down Camera - /dev/video1).
2. Melacak pergeseran piksel lantai menggunakan OpenCV Lucas-Kanade (LK).
3. Mengalikan pergeseran piksel dengan ketinggian (Z) dari Ultrasonic/Barometer.
4. Menembakkan paket MAVLink VISION_SPEED_ESTIMATE (ID 103) ke Flight Controller!
"""

import cv2
import time
import numpy as np
from pymavlink import mavutil

# =====================================================================
# 🛠️ KONFIGURASI PARAMETER (SLIPPIN' JIMMY SETUP)
# =====================================================================
MAVLINK_PORT = '/dev/ttyTHS1'  # Ganti 'udp:127.0.0.1:14550' jika test di Gazebo SITL
MAVLINK_BAUD = 57600           # Sesuaikan baudrate FC lu
CAMERA_INDEX = 1               # 0 = Kamera Depan, 1 = Kamera Bawah
SCALE_FACTOR = 0.005           # Konstanta FoV kamera (Kalibrasi lapangan: piksel ke meter)
DEFAULT_Z_HEIGHT = 1.5         # Ketinggian default jika sensor Ultrasonic belum aktif (meter)

# Parameter OpenCV Lucas-Kanade Optical Flow
LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
)

# Parameter Deteksi Titik Sudut (Shi-Tomasi Corners)
FEATURE_PARAMS = dict(
    maxCorners=100,
    qualityLevel=0.3,
    minDistance=7,
    blockSize=7
)

def connect_mavlink():
    print(f"[*] Mencoba koneksi MAVLink untuk injeksi VIO ke {MAVLINK_PORT}...")
    try:
        master = mavutil.mavlink_connection(MAVLINK_PORT, baud=MAVLINK_BAUD)
        print("[+] Koneksi MAVLink terbuka! Menunggu Heartbeat dari EKF ArduPilot...")
        # master.wait_heartbeat() # Uncomment saat di hardware fisik
        print("[✅] Heartbeat Diterima! Siap menyuap EKF satpam!")
        return master
    except Exception as e:
        print(f"[⚠️] Gagal konek MAVLink: {e}. Berjalan dalam mode Offline/Simulation Only.")
        return None

def send_vision_speed_estimate(master, vx, vy, vz=0.0):
    """
    Menembakkan paket MAVLink ID 103 (VISION_SPEED_ESTIMATE) ke ArduPilot EKF.
    vx, vy, vz dalam satuan meter/detik (m/s).
    """
    if master is None:
        return

    current_time_us = int(time.time() * 1e6)
    
    # Matriks Kovarian (Covariance Matrix): Angka kecil = Kita "meyakinkan" EKF bahwa sensor kita 100% akurat!
    covariance = [
        0.05, 0.0,  0.0,
              0.05, 0.0,
                    0.05
    ]
    
    try:
        master.mav.vision_speed_estimate_send(
            current_time_us,
            float(vx),
            float(vy),
            float(vz),
            covariance
        )
    except Exception as e:
        pass # Ignore serial write drops

def main():
    print("=========================================================")
    print("🧠 MEMULAI VIO DAEMON (LUCAS-KANADE OPTICAL FLOW INJECTOR)")
    print("=========================================================")
    
    master = connect_mavlink()
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[❌] ERROR: Kamera indeks {CAMERA_INDEX} tidak dapat dibuka!")
        print("💡 Tips: Pastikan kamera bawah tercolok, atau ubah CAMERA_INDEX ke 0/video path untuk tes.")
        return

    # Baca frame pertama sebagai referensi dasar
    ret, old_frame = cap.read()
    if not ret:
        print("[❌] Gagal membaca frame dari kamera!")
        return
        
    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **FEATURE_PARAMS)
    
    last_time = time.time()
    
    print("[🚀] TRACKING AKTIF! Mengeluarkan data kecepatan ke terminal & MAVLink...")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
                
            current_time = time.time()
            dt = current_time - last_time
            if dt <= 0:
                continue
            last_time = current_time
            
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Jika titik yang dilacak habis (misal karena blur atau gerak cepat), cari titik baru!
            if p0 is None or len(p0) < 10:
                p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **FEATURE_PARAMS)
                old_gray = frame_gray.copy()
                continue

            # Hitung Optical Flow (Lucas-Kanade)
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **LK_PARAMS)
            
            if p1 is not None and st is not None:
                # Pilih hanya titik-titik yang sukses terlacak
                good_new = p1[st == 1]
                good_old = p0[st == 1]
                
                if len(good_new) > 0 and len(good_old) > 0:
                    # Hitung rata-rata pergeseran piksel (Delta X, Delta Y)
                    shifts = good_new - good_old
                    dx_pix = np.mean(shifts[:, 0])
                    dy_pix = np.mean(shifts[:, 1])
                    
                    # -------------------------------------------------------------
                    # 🧮 RUMUS FUSI SKALA NYATA (PIXEL TO METERS PER SECOND)
                    # -------------------------------------------------------------
                    # Kecepatan (m/s) = (Pergeseran Piksel / dt) * Ketinggian Z * Scale Factor
                    # Catatan: Tanda minus (-) disesuaikan dengan orientasi kamera bawah
                    vx = -(dy_pix / dt) * DEFAULT_Z_HEIGHT * SCALE_FACTOR  # Maju/Mundur
                    vy = (dx_pix / dt) * DEFAULT_Z_HEIGHT * SCALE_FACTOR   # Kanan/Kiri
                    
                    # Filter noise kecil (Deadband) agar drone tidak bergetar saat diam
                    if abs(vx) < 0.02: vx = 0.0
                    if abs(vy) < 0.02: vy = 0.0
                    
                    # 💉 INJEKSI KE FLIGHT CONTROLLER (PHISHING EKF!)
                    send_vision_speed_estimate(master, vx, vy)
                    
                    # Logging terminal dengan frekuensi santai
                    print(f"[VIO INJECT] Vx: {vx:+05.2f} m/s | Vy: {vy:+05.2f} m/s | Titik Terlacak: {len(good_new)}")
                
                # Update memori untuk iterasi berikutnya
                old_gray = frame_gray.copy()
                p0 = good_new.reshape(-1, 1, 2)
            else:
                p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **FEATURE_PARAMS)
                old_gray = frame_gray.copy()
                
            # Sleep singkat untuk menjaga loop sekitar 20Hz - 30Hz (hemat CPU)
            time.sleep(0.03)
            
    except KeyboardInterrupt:
        print("\n[🛑] VIO Daemon dihentikan oleh pilot.")
    finally:
        cap.release()
        if master:
            master.close()
        print("[🏁] VIO Daemon Offline.")

if __name__ == "__main__":
    main()
