import asyncio
import cv2
import json
import socket
import numpy as np
import config

# =====================================================================
# MARUK FRAMEWORK: VISION DAEMON (PURE OPENCV DNN)
# Tugas: Membaca kamera fisik, inferensi AI murni tanpa Ultralytics, 
# dan kirim hasil via UDP Multicast.
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

    print(f"[VISION] Memuat Model ONNX dari {config.MODEL_PATH} via OpenCV DNN...")
    # Kita pake pure OpenCV DNN (Nggak butuh Ultralytics sama sekali)
    # Pastikan config.MODEL_PATH nembak ke file .onnx
    net = cv2.dnn.readNetFromONNX(config.MODEL_PATH)
    
    # Wajib buat Jetson Nano: Pake CUDA Backend
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    # Setup UDP Multicast Socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print("[VISION] Daemon berjalan. Memulai streaming inferensi...")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[VISION] Frame drop, mencoba lagi...")
            continue
            
        # 1. Pre-Processing YOLOv8 (Convert frame ke blob 640x640)
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (640, 640), swapRB=True, crop=False)
        net.setInput(blob)
        
        # 2. Forward Pass (Inferensi CUDA murni)
        preds = net.forward()
        
        # 3. Post-Processing Manual (Cari confidence tertinggi)
        # (Karena kita buang Ultralytics, kita kerjain ekstraksi matematisnya manual)
        preds = np.transpose(preds[0])
        
        payload = {
            "target_locked": False,
            "error_x": 0.0,
            "error_y": 0.0,
            "area": 0.0,
            "class": "None"
        }
        
        best_conf = 0
        best_box = None
        
        for row in preds:
            classes_scores = row[4:]
            _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
            class_id = max_indx[1]
            score = classes_scores[class_id]
            
            if score > config.CONFIDENCE_THRESHOLD and score > best_conf:
                best_conf = score
                # row[0:4] adalah cx, cy, w, h dalam skala 640x640
                best_box = row[0:4]
        
        # Jika ketemu target
        if best_box is not None:
            cx, cy, w, h = best_box
            
            # Skala balik ke resolusi asli (Misal kamera 640x480)
            x_scale = config.CAMERA_WIDTH / 640.0
            y_scale = config.CAMERA_HEIGHT / 640.0
            
            real_cx = cx * x_scale
            real_cy = cy * y_scale
            real_w = w * x_scale
            real_h = h * y_scale
            
            payload["target_locked"] = True
            payload["area"] = float(real_w * real_h)
            
            # Hitung Error dari titik tengah layar
            center_x = config.CAMERA_WIDTH / 2
            center_y = config.CAMERA_HEIGHT / 2
            payload["error_x"] = float(real_cx - center_x)
            payload["error_y"] = float(center_y - real_cy)
            
            # Gambar kotak di layar buat ngecek (Debug)
            x1 = int(real_cx - real_w/2)
            y1 = int(real_cy - real_h/2)
            x2 = int(real_cx + real_w/2)
            y2 = int(real_cy + real_h/2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)


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
