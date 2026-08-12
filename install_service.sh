#!/bin/bash

# Pastikan script dijalankan dengan sudo untuk bagian systemctl
echo "======================================"
echo "    MARUK AUTO-START INSTALLER 🚀     "
echo "======================================"

APP_DIR=$(pwd)
USER_NAME=$USER # Otomatis ngebaca user Jetson lu (misal 'evosky')

echo "[*] Direktori project terdeteksi: $APP_DIR"
echo "[*] Meng-generate file maruk.service..."

# Bikin file konfigurasi systemd
cat <<EOF | sudo tee /etc/systemd/system/maruk.service > /dev/null
[Unit]
Description=MARUK Autonomous Drone Engine (Vision & Autopilot)
After=network.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$APP_DIR
# Langsung tembak bash script lu
ExecStart=/bin/bash $APP_DIR/start.sh
Restart=on-failure
RestartSec=5
# Log bisa dicek pakai 'journalctl -u maruk.service' kalau lagi colok kabel
StandardOutput=syslog
StandardError=syslog
SyslogIdentifier=maruk-ai

[Install]
WantedBy=multi-user.target
EOF

echo "[*] Mendaftarkan service ke sistem Ubuntu Jetson..."
sudo systemctl daemon-reload
sudo systemctl enable maruk.service

echo "======================================"
echo "[SUCCESS] INSTALASI BERHASIL! 🎉"
echo "======================================"
echo "Sekarang, setiap kali Jetson lu dicolok baterai (nyala),"
echo "script start.sh bakal otomatis jalan di background!"
echo ""
echo "Cara tes manual (tanpa restart):"
echo "  sudo systemctl start maruk.service"
echo ""
echo "Cara matiin (kalau lagi maintenance):"
echo "  sudo systemctl stop maruk.service"
echo "======================================"
