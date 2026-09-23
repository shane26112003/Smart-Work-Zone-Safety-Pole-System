#!/bin/bash
# ==============================================================================
# Smart Work-Zone Safety Pole System - Automated Raspberry Pi 5 Setup Script
# Run this script on your Raspberry Pi 5 running Raspberry Pi OS (Bookworm 64-bit)
# Usage: chmod +x scripts/setup_pi.sh && ./scripts/setup_pi.sh
# ==============================================================================

set -e

echo "=================================================================="
echo " Setting up Smart Work-Zone Safety Pole System on Raspberry Pi 5 "
echo "=================================================================="

# 1. Update OS Packages
echo "[1/6] Updating system repositories..."
sudo apt update && sudo apt upgrade -y

# 2. Install System Dependencies & Hardware Libraries
echo "[2/6] Installing system packages, OpenCV, Libcamera, and build tools..."
sudo apt install -y \
    python3-pip \
    python3-venv \
    python3-numpy \
    python3-scipy \
    python3-opencv \
    python3-tornado \
    python3-serial \
    python3-yaml \
    python3-picamera2 \
    python3-libcamera \
    libcamera-tools \
    v4l-utils \
    git \
    sqlite3

# 3. Add User to Hardware Peripheral Groups
echo "[3/6] Configuring hardware permissions (gpio, video, dialout)..."
sudo usermod -aG video,dialout,gpio $USER

# 4. Create Python Virtual Environment with System Packages
echo "[4/6] Creating Python virtual environment..."
cd "$(dirname "$0")/.."
if [ ! -d "venv" ]; then
    python3 -m venv --system-site-packages venv
    echo "Virtual environment created at ./venv"
fi

source venv/bin/activate

# 5. Install PyTorch & Ultralytics YOLO
echo "[5/6] Installing Ultralytics YOLO and PyTorch for ARM64..."
pip install --upgrade pip
pip install ultralytics torch torchvision

# 6. Verify Installation
echo "[6/6] Running system verification tests..."
python tests/run_all_tests.py

echo "=================================================================="
echo " SETUP COMPLETE! "
echo " Next steps:"
echo " 1. Configure Pi 5 Wi-Fi Hotspot (SAFETY_POLE_AP) or connect to WiFi."
echo " 2. Flash ESP32-S3 Worker tag and ESP32 LiDAR firmware."
echo " 3. Launch system: source venv/bin/activate && python main.py --mode hardware"
echo " 4. Open dashboard at: http://<PI_IP>:8080"
echo "=================================================================="
