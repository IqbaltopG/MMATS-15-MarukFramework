#!/bin/bash

# =====================================================================
# Script Otomatis Setup MARUK Framework di Jetson Nano
# Dibuat khusus untuk menghindari lupa flag --system-site-packages!
# =====================================================================

echo "🚀 [MARUK] Memulai Setup Environment untuk Jetson Nano..."

echo "📦 [1/3] Membuat Virtual Environment khusus Jetson..."
# WAJIB: --system-site-packages agar OpenCV (CUDA) & TensorRT bawaan NVIDIA tidak pecah
python3 -m venv venv --system-site-packages

echo "🔄 [2/3] Mengaktifkan Virtual Environment..."
# Kita pakai 'source' langsung di script
source venv/bin/activate

echo "⬇️  [3/3] Menginstall Dependensi dari requirements.txt..."
# Install PyMavlink, Ultralytics, PySerial
pip install -r requirements.txt

echo "====================================================================="
echo "✅ SETUP BERHASIL, BOS!"
echo "⚠️  Ingat: Tiap buka terminal baru, lu WAJIB jalanin perintah ini dulu:"
echo "👉 source venv/bin/activate"
echo "====================================================================="
