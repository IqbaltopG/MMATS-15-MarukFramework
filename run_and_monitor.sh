#!/bin/bash

echo "🚀 Menjalankan MARUK Autopilot di Background..."
python3 autopilot.py &
AUTOPILOT_PID=$!

# echo "👁️ Menjalankan Vision Daemon di Background..."
# python3 vision_daemon.py &
# VISION_PID=$!
echo "⚠️ Vision Daemon (YOLO) di-disable sementara untuk tes State Machine murni."
echo "⚠️ (YOLO butuh konversi TensorRT .engine dulu biar Jetson gak meledak)."

echo "======================================================"
echo "✅ Sistem berjalan! Membuka HTOP untuk monitor CPU/RAM."
echo "Tekan 'q' untuk keluar dari htop, lalu skrip akan mematikan Python otomatis."
echo "======================================================"
sleep 2

# Buka htop dengan filter khusus process python biar gampang liatnya
htop -p $AUTOPILOT_PID

echo "🛑 HTOP ditutup. Mematikan semua proses MARUK..."
kill $AUTOPILOT_PID
echo "✅ Semua proses dimatikan. Bersih!"
