import cv2
import time
import os
import signal
import sys
import config

# =====================================================================
# SCRIPT KOLEKSI DATASET FISIK (Passive POV Recording)
# Tujuan: Merekam video asli saat drone terbang manual (Pilot by Human)
# =====================================================================

is_recording = True

def handle_sigint(sig, frame):
    global is_recording
    print("\n[REC] Sinyal Ctrl+C diterima. Menyimpan video...")
    is_recording = False

# Tangkap sinyal Ctrl+C agar file video tidak corrupt saat distop paksa
signal.signal(signal.SIGINT, handle_sigint)

def main():
    print(f"[REC] Mengakses Kamera Depan (Index {config.CAMERA_FRONT})...")
    cap_front = cv2.VideoCapture(config.CAMERA_FRONT)
    print(f"[REC] Mengakses Kamera Bawah (Index {config.CAMERA_DOWN})...")
    cap_down = cv2.VideoCapture(config.CAMERA_DOWN)
    
    if not cap_front.isOpened():
        print(f"[REC] ERROR: Gagal membuka kamera DEPAN ({config.CAMERA_FRONT})!")
        
    if not cap_down.isOpened():
        print(f"[REC] ERROR: Gagal membuka kamera BAWAH ({config.CAMERA_DOWN})!")
        
    if not cap_front.isOpened() and not cap_down.isOpened():
        print("[REC] KEDUA KAMERA GAGAL DIBUKA! Cek kabel CSI/USB Jetson.")
        return

    # Set Resolusi Kamera sesuai config
    for cap in [cap_front, cap_down]:
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

    # Siapkan folder dataset jika belum ada
    os.makedirs("dataset", exist_ok=True)
    
    # Bikin nama file unik berdasarkan waktu
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    out_front = None
    out_down = None
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    
    if cap_front.isOpened():
        out_front = cv2.VideoWriter(f"dataset/FRONT_POV_{timestamp}.mp4", fourcc, config.CAMERA_FPS, (config.CAMERA_WIDTH, config.CAMERA_HEIGHT))
    if cap_down.isOpened():
        out_down = cv2.VideoWriter(f"dataset/DOWN_POV_{timestamp}.mp4", fourcc, config.CAMERA_FPS, (config.CAMERA_WIDTH, config.CAMERA_HEIGHT))

    print(f"[REC] =========================================")
    print(f"[REC] MEREKAM VIDEO KE FOLDER 'dataset/'")
    print(f"[REC] Kamera Aktif: {'DEPAN ' if cap_front.isOpened() else ''}{'BAWAH' if cap_down.isOpened() else ''}")
    print(f"[REC] Resolusi: {config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT} @ {config.CAMERA_FPS} FPS")
    print(f"[REC] TEKAN 'Ctrl+C' DI TERMINAL UNTUK BERHENTI!")
    print(f"[REC] =========================================")

    frame_count = 0
    start_time = time.time()

    while is_recording:
        if cap_front.isOpened():
            ret_f, frame_f = cap_front.read()
            if ret_f and out_front:
                out_front.write(frame_f)
                
        if cap_down.isOpened():
            ret_d, frame_d = cap_down.read()
            if ret_d and out_down:
                out_down.write(frame_d)
                
        frame_count += 1
        
        # Cetak status setiap 30 frame (~1 detik)
        if frame_count % config.CAMERA_FPS == 0:
            elapsed = int(time.time() - start_time)
            print(f"[REC] Durasi rekam: {elapsed} detik | Frames: {frame_count}", end='\r')

    # Bersihkan memori dan tutup file
    print("\n[REC] Menyelesaikan proses rekaman...")
    if cap_front.isOpened(): cap_front.release()
    if cap_down.isOpened(): cap_down.release()
    if out_front: out_front.release()
    if out_down: out_down.release()
    print(f"[REC] ✅ Video berhasil disimpan di folder 'dataset/'")

if __name__ == "__main__":
    main()
