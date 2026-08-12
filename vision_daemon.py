import cv2
import socket
import json
import numpy as np
import time
import subprocess
import re
import threading
import os
import sys
from ultralytics import YOLO
import supervision as sv

# CONFIGURATION
UDP_IP = "127.0.0.1"
MODEL_PATH = "yolov8n.pt" # Pakai model bawaan YOLO (COCO) sementara buat Ghost Flight
CONFIDENCE_THRESHOLD = 0.25

import config

# GStreamer Pipeline dihapus karena kita pakai USB Webcam
FRONT_CAM = config.CAMERA_FRONT
DOWN_CAM  = config.CAMERA_DOWN

# UDP CONFIG
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def open_camera_with_timeout(pipeline, timeout_sec=60):
    result = [None]
    def _open():
        cap = cv2.VideoCapture(pipeline)
        if not cap.isOpened():
            return
        start_t = time.time()
        while time.time() - start_t < timeout_sec:
            ret, frame = cap.read()
            if ret and frame is not None:
                result[0] = cap
                return
            time.sleep(0.1)
    t = threading.Thread(target=_open, daemon=True)
    t.start()
    t.join(timeout=timeout_sec)
    if t.is_alive() or result[0] is None:
        print(f"[VISION] ⏰ Camera timeout setelah {timeout_sec}s! Stream belum ada atau buffering.")
        return None
    return result[0]

def start_vision_daemon():
    print(f"[VISION] Loading model: {MODEL_PATH} (CPU Mode - Skip CUDA)")
    model = YOLO(MODEL_PATH)
    model.to('cpu')  # Force CPU, skip CUDA drama!

    print(f"[VISION] Membuka kamera front (Index {FRONT_CAM})... max 60 detik")
    cap_front = open_camera_with_timeout(FRONT_CAM, timeout_sec=60)
    
    print(f"[VISION] Membuka kamera down (Index {DOWN_CAM})... max 60 detik")
    cap_down = open_camera_with_timeout(DOWN_CAM, timeout_sec=60)


    if cap_front is None:
        print("[VISION] FATAL: Kamera FRONT gagal! Pastiin ArduPilot + Gazebo udah jalan.")
        sys.exit(1)
    if cap_down is None:
        print("[VISION] FATAL: Kamera DOWN gagal! Pastiin ArduPilot + Gazebo udah jalan.")
        sys.exit(1)

    print(f"[VISION] ✅ Mata terbuka! Mengirim data ke {UDP_IP}:{UDP_PORT}...")

    # SUPERVISION: Dual Tracker diaktifkan kembali!
    tracker_front = sv.ByteTrack()
    tracker_down = sv.ByteTrack()

    use_front = True # Toggle untuk alternating frames

    try:
        while True:
            # Alternating frames: Frame ganjil = Depan, Genap = Bawah (hemat CPU)
            if use_front:
                ret, frame = cap_front.read()
                cam_name = "front"
                active_tracker = tracker_front
            else:
                ret, frame = cap_down.read()
                cam_name = "down"
                active_tracker = tracker_down
                
            use_front = not use_front

            if not ret:
                time.sleep(0.05)
                continue
            
            height, width, _ = frame.shape
            frame_center_x = width // 2
            frame_center_y = height // 2

            results = model.predict(source=frame, conf=CONFIDENCE_THRESHOLD, verbose=False)[0]
            
            # SUPERVISION: Konversi ke objek Detections murni pake Numpy
            raw_detections = sv.Detections.from_ultralytics(results)
            
            # Coba masukin ke ByteTrack buat di-smoothing & prediksi
            tracked_detections = active_tracker.update_with_detections(raw_detections)
            
            # THE "SLIPPIN' JIMMY" WORKAROUND (Hybrid Vision):
            # Kalo drone lagi muter kenceng (Tawaf), ByteTrack bakal nge-drop objek karena IoU 0.
            # Kalo tracker kosong TAPI mata asli (YOLO) ngeliat objeknya, kita BYPASS pake mata asli!
            # Nanti pas drone udah ngerem (LOCKED), ByteTrack bakal otomatis ngambil alih lagi.
            if len(tracked_detections) > 0:
                final_detections = tracked_detections
            else:
                final_detections = raw_detections
            
            target_data = {"status": "LOST", "class": "none", "error_x": 0, "error_y": 0, "area": 0, "camera": cam_name}

            if len(final_detections) > 0:
                # SUPERVISION: Ambil xyxy sebagai NumPy Array (N, 4)
                xyxys = final_detections.xyxy
                
                # CLAMPING: Cegah bounding box tumpah ke luar layar pake clip NumPy (Anti X = -907)
                xyxys[:, 0] = np.clip(xyxys[:, 0], 0, width)
                xyxys[:, 1] = np.clip(xyxys[:, 1], 0, height)
                xyxys[:, 2] = np.clip(xyxys[:, 2], 0, width)
                xyxys[:, 3] = np.clip(xyxys[:, 3], 0, height)
                
                # Hitung Centroid X untuk semua object sekalian pake Vectorization!
                centroids_x = (xyxys[:, 0] + xyxys[:, 2]) / 2.0
                err_xs = np.abs(centroids_x - frame_center_x)
                
                # PILIH TARGET PALING TENGAH (Bukan Paling Gede)
                best_idx = np.argmin(err_xs)
                
                best_xyxy = xyxys[best_idx]
                cls_id = final_detections.class_id[best_idx]
                class_name = model.names[cls_id]
                
                # Ambil Tracker ID kalo ada
                trk_id = -1
                if final_detections.tracker_id is not None:
                    trk_id = int(final_detections.tracker_id[best_idx])
                
                x1, y1, x2, y2 = best_xyxy
                centroid_x = (x1 + x2) / 2.0
                centroid_y = (y1 + y2) / 2.0
                
                error_x = centroid_x - frame_center_x
                error_y = centroid_y - frame_center_y
                max_area = (x2 - x1) * (y2 - y1)

                target_data = {
                    "status": "LOCKED",
                    "class": class_name,
                    "tracker_id": trk_id,
                    "error_x": int(error_x),
                    "error_y": int(error_y),
                    "area": int(max_area),
                    "camera": cam_name
                }
                
            if target_data["status"] == "LOCKED":
                tid = target_data.get('tracker_id', -1)
                print(f"[DEBUG VISION] [{cam_name.upper()}] YOLO ngeliat: {target_data['class']} (ID:{tid}) | X={target_data['error_x']}")
                            
            message = json.dumps(target_data).encode('utf-8')
            sock.sendto(message, (UDP_IP, UDP_PORT))

            time.sleep(0.05) # Loop stabil di ~20 FPS

    except KeyboardInterrupt:
        print("\n[VISION] Daemon dihentikan.")
    finally:
        cap_front.release()
        if cap_down is not None:
            cap_down.release()
        sock.close()

if __name__ == "__main__":
    # Lidar dihapus sesuai keterbatasan dunia nyata!
    
    # Start Vision Camera (Foreground)
    start_vision_daemon()
