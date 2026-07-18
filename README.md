# REAL_DRONE Deployment Environment

Folder ini disiapkan khusus untuk **Deployment Fisik (Sim-to-Real)** di atas Jetson Orin Nano / Raspberry Pi 5 yang terhubung langsung ke wahana DekeFPV H743.

## Rencana Development (Sim-to-Real):
1. **Config Terpusat (`config.py`)**: Semua nilai konstan (Tuning KP, Baudrate Serial, Index Kamera, Timeout Threshold) bakal dipindahin ke satu file ini biar gampang di-tweak di lapangan tanpa harus ngobrak-ngabrik `states.py`.
2. **Kamera Fisik (`vision_daemon.py`)**: Mengganti input kamera dari `UDP/GStreamer` Gazebo menjadi murni `cv2.VideoCapture('/dev/video0')` dan `pyrealsense2` (VPU).
3. **Koneksi Serial (`comms.py`)**: Mengganti MAVLink UDP `127.0.0.1:14550` menjadi UART Serial `/dev/ttyAMA0` baudrate `921600`.
4. **Export Model AI**: Konversi model `.pt` menjadi `.engine` (TensorRT) untuk memaksimalkan performa Ampere GPU di Jetson Orin Nano.
5. **Operational Failsafes**: 
   - Mapping RC Channel 5 untuk *Pause & Resume* (ALT_HOLD ↔ GUIDED).
   - Menambahkan fungsi *Emergency RTB* (RTL / Land) di `states.py` jika YOLO mengalami kebutaan (`timeout_counter > 300`).

> **Catatan:** Jangan pernah campur file dari folder `DRONE_ARDU` (SITL) ke folder ini tanpa memisahkan parameternya ke `config.py` terlebih dahulu!
