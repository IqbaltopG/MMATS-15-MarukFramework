import asyncio
import cv2
import json
import socket
import config
from ultralytics import YOLO

# =====================================================================
# MARUK FRAMEWORK: VISION DAEMON (SENSE LAYER)
# Tugas: Membaca kamera fisik, inferensi AI, dan kirim hasil via UDP Multicast.
# =====================================================================

def main():
    print(f"[VISION] Inisialisasi Kamera di index {config.CAMERA_FRONT}...")
    cap = cv2.VideoCapture(config.CAMERA_FRONT)
    
    if not cap.isOpened():
        print("[VISION] ERROR: Gagal membuka kamera fisik!")
        return

    # Set Resolusi Kamera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

    print(f"[VISION] Memuat Model AI dari {config.MODEL_PATH}...")
    # NOTE: Untuk TensorRT, pastikan MODEL_PATH mengarah ke file .engine
    model = YOLO(config.MODEL_PATH)

    # Setup UDP Multicast Socket (Fire and Forget)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print("[VISION] Daemon berjalan. Memulai streaming inferensi...")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[VISION] Frame drop, mencoba lagi...")
            continue
            
        # Inferensi YOLO
        results = model.predict(source=frame, conf=config.CONFIDENCE_THRESHOLD, verbose=False)
        
        # Ekstraksi Bounding Box (Kosongan / Framework Mode)
        # TODO: Tambahkan logika spesifik untuk mem-parsing bounding box jadi koordinat error_x, error_y
        payload = {
            "target_locked": False,
            "error_x": 0.0,
            "error_y": 0.0,
            "area": 0.0,
            "class": "None"
        }
        
        if len(results) > 0 and len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            # Dummy perhitungan centroid
            payload["target_locked"] = True
            payload["class"] = model.names[int(box.cls[0])]
            payload["area"] = float(box.xywh[0][2] * box.xywh[0][3])

        # Kirim data ke Autopilot via UDP (Port 5005)
        udp_socket.sendto(json.dumps(payload).encode('utf-8'), ('127.0.0.1', 5005))
        
        # Tampilkan Window Kamera (Matikan saat turnamen resmi)
        cv2.imshow("MARUK Vision Daemon", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
