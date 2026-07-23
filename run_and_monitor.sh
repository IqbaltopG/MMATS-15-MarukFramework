#!/bin/bash

echo "🚀 Menjalankan MARUK Autopilot di Background..."
python3 autopilot.py &
AUTOPILOT_PID=$!

echo "👁️ Menjalankan Vision Daemon di Background..."
python3 vision_daemon.py &
VISION_PID=$!

echo "======================================================"
echo "✅ Sistem berjalan! Membuka HTOP untuk monitor CPU/RAM."
echo "Tekan 'q' untuk keluar dari htop, lalu skrip akan mematikan Python otomatis."
echo "======================================================"
sleep 2

# Buka htop dengan filter khusus process python biar gampang liatnya
htop -p $AUTOPILOT_PID,$VISION_PID

echo "🛑 HTOP ditutup. Mematikan semua proses MARUK..."
kill $AUTOPILOT_PID
kill $VISION_PID
echo "✅ Semua proses dimatikan. Bersih!"
