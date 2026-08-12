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
    
    # Bikin nama folder unik berdasarkan waktu
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    save_dir = f"dataset/run_{timestamp}"
    os.makedirs(save_dir, exist_ok=True)
    
    # FPS target buat nyimpen dataset (misal 5 foto per detik biar dapet banyak angle)
    TARGET_SAVE_FPS = 5
    frame_interval = max(1, config.CAMERA_FPS // TARGET_SAVE_FPS)

    print(f"[REC] =========================================")
    print(f"[REC] MEREKAM GAMBAR KE FOLDER: {save_dir}")
    print(f"[REC] Kamera Aktif: {'DEPAN ' if cap_front.isOpened() else ''}{'BAWAH' if cap_down.isOpened() else ''}")
    print(f"[REC] Resolusi: {config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT} @ {TARGET_SAVE_FPS} FPS")
    print(f"[REC] =========================================")

    frame_count = 0
    saved_count = 0
    start_time = time.time()
    last_save_time = time.time()
    SAVE_INTERVAL = 1.0 / TARGET_SAVE_FPS  # 0.2 detik per frame

    while is_recording:
        ret_f, frame_f = False, None
        ret_d, frame_d = False, None
        
        if cap_front.isOpened():
            ret_f, frame_f = cap_front.read()
                
        if cap_down.isOpened():
            ret_d, frame_d = cap_down.read()
                
        frame_count += 1
        
        # Simpan gambar berdasarkan REAL-TIME bukan frame count (Loop bisa jalan >1000fps!)
        now = time.time()
        if now - last_save_time >= SAVE_INTERVAL:
            last_save_time = now
            if ret_f and frame_f is not None:
                cv2.imwrite(f"{save_dir}/front_{saved_count}.jpg", frame_f)
            if ret_d and frame_d is not None:
                cv2.imwrite(f"{save_dir}/down_{saved_count}.jpg", frame_d)
                
            saved_count += 1
            elapsed = int(time.time() - start_time)
            print(f"[REC] Waktu berjalan: {elapsed}s | Foto tersimpan: {saved_count} pasang", end='\r')
        
        time.sleep(0.01)  # Anti CPU 100%, yield ke OS

    # Bersihkan memori
    print("\n[REC] Menyelesaikan proses rekaman...")
    if cap_front.isOpened(): cap_front.release()
    if cap_down.isOpened(): cap_down.release()
    print(f"[REC] ✅ {saved_count} pasang foto berhasil disimpan di folder '{save_dir}'")

if __name__ == "__main__":
    main()
