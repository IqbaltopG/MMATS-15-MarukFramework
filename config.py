# config.py — MASTER CONFIGURATION (Real Hardware)

# === ENVIRONMENT ===
IS_SIMULATION = False

# === FC CONNECTION (DekeFPV) ===
FC_CONNECTION = '/dev/ttyTHS1'  # UART port pada Jetson Nano
FC_BAUD = 921600                # Baudrate tinggi untuk komunikasi MAVLink

# === CAMERA (Vision Daemon) ===
# Ganti dengan 0 untuk USB Webcam atau '/dev/video0'
CAMERA_FRONT = 0
CAMERA_DOWN = 1
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# === AI / YOLO ===
# Jika sudah diconvert ke TensorRT, ubah ini ke 'model.engine'
MODEL_PATH = 'Krti_model.pt' 
CONFIDENCE_THRESHOLD = 0.55

# === TUNING KINEMATICS (Proportional Navigation) ===
# Nilai-nilai ini adalah "Position Sizing" untuk membatasi Throttle < 60% (Anti-Brownout)
KP_YAW = 0.05
KP_UP = 0.05
FWD_MAX_SPEED = 1.2     # Jangan lebih dari 2.0 m/s biar FC ga nunduk ekstrem
STRAFE_MAX_SPEED = 0.25 # Pelan aja buat ngepasin bolongan gawang
UP_MAX_SPEED = 0.5      # [PENTING] Clamp power vertikal (Motor nyedot Ampere terbesar pas naik!)
DOWN_MAX_SPEED = -0.3   # Turun pelan biar ga kehilangan daya angkat (Vortex Ring State)

# === STATE MACHINE TIMEOUTS ===
# Di Gazebo (RTF lambat), timeout 100 ticks = 10 detik.
# Di Real Life (RTF 100%), timeout harus dikurangi setengahnya agar responsif!
TIMEOUT_MULTIPLIER = 1.0 # Ubah ke 1.0 untuk Real Life, 2.0 untuk Gazebo
