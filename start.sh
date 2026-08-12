#!/bin/bash
echo "======================================"
echo "    MARUK FRAMEWORK LAUNCHER 🚀       "
echo "======================================"

# Masuk ke virtual environment otomatis
source venv/bin/activate

echo "[1/2] Menyalakan Mata AI (vision_daemon.py) di background..."
python3 vision_daemon.py &
VISION_PID=$!

echo "[*] Menunggu 5 detik biar kamera panas dan YOLO siap..."
sleep 5

echo "[2/2] Menyalakan Otak Autopilot (autopilot.py)..."
python3 autopilot.py
AUTOPILOT_PID=$!

# Fungsi buat matiin semua sistem kalau lu pencet Ctrl+C
trap "echo -e '\n[!] Mematikan Sistem MARUK...'; kill $VISION_PID; exit" INT

wait $AUTOPILOT_PID
kill $VISION_PID
echo "[MARUK] Sistem Selesai."
