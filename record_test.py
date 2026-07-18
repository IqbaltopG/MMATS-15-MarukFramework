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
    print(f"[REC] Mengakses Kamera di index {config.CAMERA_FRONT}...")
    cap = cv2.VideoCapture(config.CAMERA_FRONT)
    
    if not cap.isOpened():
        print(f"[REC] ERROR: Gagal membuka kamera {config.CAMERA_FRONT}! Cek kabel/konektor CSI/USB.")
        return

    # Set Resolusi Kamera sesuai config
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

    # Siapkan folder dataset jika belum ada
    os.makedirs("dataset", exist_ok=True)
    
    # Bikin nama file unik berdasarkan waktu
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    output_filename = f"dataset/flight_pov_{timestamp}.mp4"
    
    # Setup Video Writer (Format MP4, Codec mp4v)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_filename, fourcc, config.CAMERA_FPS, (config.CAMERA_WIDTH, config.CAMERA_HEIGHT))

    print(f"[REC] =========================================")
    print(f"[REC] MEREKAM VIDEO KE: {output_filename}")
    print(f"[REC] Resolusi: {config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT} @ {config.CAMERA_FPS} FPS")
    print(f"[REC] TEKAN 'Ctrl+C' DI TERMINAL UNTUK BERHENTI!")
    print(f"[REC] =========================================")
    print("[REC] Silakan Pilot terbang manual sekarang...")

    frame_count = 0
    start_time = time.time()

    while is_recording:
        ret, frame = cap.read()
        if not ret:
            print("[REC] Frame drop atau kamera terputus!")
            break
            
        # Tulis frame ke file mp4
        out.write(frame)
        frame_count += 1
        
        # Cetak status setiap 30 frame (~1 detik)
        if frame_count % config.CAMERA_FPS == 0:
            elapsed = int(time.time() - start_time)
            print(f"[REC] Durasi rekam: {elapsed} detik | Frames: {frame_count}", end='\r')

    # Bersihkan memori dan tutup file
    print("\n[REC] Menyelesaikan proses rekaman...")
    cap.release()
    out.release()
    print(f"[REC] ✅ Video berhasil disimpan di {output_filename}")
    print(f"[REC] Silakan pindahkan file ini ke laptop untuk dianotasi di Roboflow.")

if __name__ == "__main__":
    main()
