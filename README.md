# Smart Work-Zone Safety Pole System

An AI-powered smart safety pole system for road construction and work zones, designed to prevent vehicle-worker collisions through multi-sensor fusion (**Dual Camera Computer Vision + TSD20 LiDAR + ESP32-S3 Worker Wearable IMU & BLE**), real-time trajectory prediction, and instant multi-modal warnings, deployed on **Raspberry Pi 5**.

---

## 1. System Architecture

```
                                  ┌───────────────────────────┐
                                  │   TSD20 LiDAR (Serial)    │
                                  │  Approaching Vehicle Depth│
                                  └─────────────┬─────────────┘
                                                │
┌───────────────────────────────┐               │               ┌───────────────────────────────┐
│     ESP32-S3 Worker Tag       │               │               │   Raspberry Pi Camera & USB   │
│  MPU6500 IMU (Accel/Gyro/SVM) │               │               │    Dual Video Feeds (Road &   │
│   BLE Beacon & Wi-Fi UDP      │               │               │           Work-Zone)          │
└───────────────┬───────────────┘               │               └───────────────┬───────────────┘
                │ Wi-Fi / BLE                   │ Wi-Fi UDP                     │ MIPI-CSI / USB
                ▼                               ▼                               ▼
       ┌─────────────────┐             ┌─────────────────┐             ┌─────────────────┐
       │ Worker Telemetry│             │  LiDAR Driver   │             │ YOLOv8 Detector │
       │    Receiver     │             │  & Range Filter │             │   & DeepSORT    │
       └────────┬────────┘             └────────┬────────┘             └────────┬────────┘
                │                               │                               │
                └───────────────────────┬───────┴───────────────────────────────┘
                                        │
                                        ▼
                             ┌─────────────────────┐
                             │ SENSOR FUSION ENGINE│
                             │  - EKF State Filter │
                             │  - BEV Homography   │
                             │  - ID & Spatial Map │
                             └──────────┬──────────┘
                                        │
                                        ▼
                             ┌─────────────────────┐
                             │ COLLISION RISK      │
                             │ ASSESSMENT ENGINE   │
                             │  - TTC / CPA Model  │
                             │  - Trajectory Cone  │
                             │  - IMU State Weight │
                             │  - 4-Tier Risk Eval │
                             └──────────┬──────────┘
                                        │
                         ┌──────────────┴──────────────┐
                         ▼                             ▼
              ┌─────────────────────┐       ┌─────────────────────┐
              │   WARNING SYSTEM    │       │ DIGITAL TWIN WEB    │
              │  - Directional LEDs │       │ DASHBOARD & LOGGER  │
              │  - Smart Siren/Relay│       │  - Live Dual Video  │
              │  - Audio Alarm Buzzer│      │  - 2D Radar BEV Map │
              └─────────────────────┘       │  - Incident Telemetry│
                                            └─────────────────────┘
```

---

## 2. Key Components

### 2.1 Central Unit (Raspberry Pi 5)
- **Processor**: Broadcom BCM2712 Quad-core Arm Cortex-A76 @ 2.4GHz with 4GB/8GB LPDDR4X RAM.
- **Cameras**:
  - Road-Facing Camera: Raspberry Pi Camera Module 3 (Wide) via MIPI CSI ribbon cable.
  - Work-Zone Area Camera: USB 3.0 HD Webcam.
- **Actuators**: Directional High-Intensity LED flashers (GPIO 17, 22), 12V Siren Relay (GPIO 27), and Active Piezo Buzzer (GPIO 23).
- **Network**: Operates as Wi-Fi Access Point (`SAFETY_POLE_AP`) receiving incoming sensor telemetry.

### 2.2 Worker Module (ESP32-S3 + MPU6500)
- Wearable badge powered by a 3.7V LiPo battery.
- **MPU6500 6-Axis IMU**: Measures acceleration ($a_x, a_y, a_z$) in $g$, angular rate ($\omega_x, \omega_y, \omega_z$) in $\text{deg/s}$, and Signal Vector Magnitude $\text{SVM} = \sqrt{a_x^2 + a_y^2 + a_z^2}$.
- **Motion Classifier**: Classifies movement into `STATIC`, `WALKING`, `RUNNING`, `IMPACT`, and `FALL_DETECTED`.
- **Dual Telemetry**:
  - Wi-Fi UDP client streaming telemetry JSON at 25 Hz to RPi 5 port `5005`.
  - BLE Beacon advertising Worker ID and status for RSSI proximity estimation.
  - Receives haptic feedback packets from RPi 5 when danger is detected.

### 2.3 Road LiDAR (TSD20 ToF LiDAR + ESP32 Bridge)
- Long-range time-of-flight LiDAR aimed along the road approach line.
- Measures closing vehicle distance (0.1m - 20m) and range rate ($\dot{r}$).
- Transmits data at 50 Hz over Wi-Fi UDP to RPi 5 port `5006` (or direct UART).

### 2.4 AI Computer Vision & Tracking
- **Detector**: High-precision YOLOv8 / YOLO11 (`yolov8s.pt`, 44.9 mAP, with automatic fallbacks to `yolo11s.pt` or `yolov8n.pt`) with real-time PyTorch CPU 4-thread execution for Raspberry Pi 5.
- **Worker PPE Verifier**: Multi-spectral color and feature classifier analyzing upper body crops for ANSI Class 2/3 high-visibility safety vests (fluorescent orange / lime green / reflective silver) and safety hard hats.
- **Hardware Image Enhancer**: Adaptive CLAHE (Contrast-Limited Adaptive Histogram Equalization) and 4-channel BGRA/XBGR normalization designed specifically for the Raspberry Pi Camera Module Rev 1.3 (OV5647).
- **Tracker**: Multi-Object Tracking with Kalman Filter and Hungarian IoU association, computing object velocity vectors, maintaining persistent IDs, and tracking PPE compliance.

### 2.5 Multi-Sensor Extended Kalman Filter (EKF) Fusion
- Transforms camera pixels to Bird's-Eye View (BEV) ground metric coordinates $(X, Y)$ in meters.
- Fuses visual vehicle tracks with LiDAR millimeter depth and range rate.
- Associates visual worker detections with active ESP32 tags using spatial proximity and IMU motion correlation.

### 2.6 Collision-Risk Assessment Engine
- Computes Closest Point of Approach (CPA) time ($t_{\text{CPA}}$) and miss distance ($d_{\text{CPA}}$).
- Calculates Time-To-Collision (TTC).
- Evaluates worker motion state (e.g. fallen worker cannot dodge).
- Four-tier classification:
  - `SAFE`: Normal traffic in lane, workers within buffer zone.
  - `CAUTION`: Vehicle approaching within 30m, TTC < 7.0s, or worker near lane line.
  - `HIGH RISK`: Converging trajectory within 3m, TTC < 4.5s.
  - `CRITICAL`: Imminent trajectory intersection, TTC < 2.5s, or worker down in path.

---

## 3. Directory Structure

```
safety/
├── firmware/
│   ├── esp32_worker/                 # Worker Wearable Tag Firmware
│   │   ├── esp32_worker.ino          # Arduino sketch (MPU6500, BLE, WiFi UDP)
│   │   └── config.h                  # Worker ID, WiFi credentials, pins
│   └── esp32_lidar/                  # TSD20 LiDAR WiFi Bridge Firmware
│       ├── esp32_lidar.ino           # Arduino sketch (UART2 reader, WiFi UDP)
│       └── config.h                  # LiDAR pins, baud rate, ports
├── hardware/
│   ├── schematics.md                 # Full wiring schematics & pinout guide
│   └── bill_of_materials.md         # Component list & specifications
├── src/
│   ├── config.py                     # Master system configuration
│   ├── sensors/
│   │   ├── lidar_tsd20.py            # TSD20 LiDAR driver (WiFi UDP / Serial / Mock)
│   │   ├── worker_receiver.py        # ESP32 worker telemetry receiver (UDP 5005)
│   │   └── camera_manager.py         # Dual camera capture (CSI, USB, Synthetic)
│   ├── cv/
│   │   ├── detector.py               # YOLOv8 object detector
│   │   └── tracker.py                # Multi-object Kalman Filter tracker
│   ├── fusion/
│   │   ├── bev_transform.py          # Perspective-to-BEV homography mapper
│   │   └── ekf_fusion.py             # Multi-sensor Extended Kalman Filter
│   ├── risk/
│   │   └── risk_engine.py            # TTC, CPA & 4-tier collision risk engine
│   ├── warning/
│   │   ├── alert_controller.py       # Hardware GPIO strobe & siren controller
│   │   └── event_logger.py           # SQLite & JSONL black-box incident recorder
│   └── dashboard/
│       ├── server.py                 # Tornado async web & WebSocket server
│       └── static/
│           ├── index.html            # Tactical digital twin UI
│           ├── app.js                # HTML5 canvas radar & Web Audio buzzer
│           └── style.css             # High-contrast industrial dark theme
├── simulation/
│   └── scenario_simulator.py         # Interactive multi-agent traffic simulator
├── scripts/
│   ├── setup_pi.sh                   # Automated Raspberry Pi 5 installer
│   ├── setup_hotspot.sh              # Network hotspot configuration script
│   └── test_model.py                 # YOLO accuracy & PPE benchmark tool
├── tests/
│   ├── test_cv_detector.py           # YOLO detector, CLAHE & PPE tests
│   ├── test_imu_processing.py        # IMU features & fall detection tests
│   ├── test_lidar_driver.py          # TSD20 packet parsing & range tests
│   ├── test_ekf_fusion.py            # BEV transform & EKF tests
│   ├── test_risk_engine.py           # Collision risk & TTC tests
│   ├── test_system_pipeline.py       # Full end-to-end integration test
│   └── run_all_tests.py              # Automated test suite runner
├── main.py                           # Primary system entry point
├── run_simulation.py                 # Quick one-click simulation launcher
├── requirements.txt                  # Python dependencies
└── README.md                         # Documentation
```

---

## 4. Complete Command Reference & Operations Guide

### 4.1 Raspberry Pi SSH Key Regeneration & Connection

When setting up a fresh Raspberry Pi OS image, re-imaging the SD card, or when your client computer reports `WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!`, regenerate the SSH host keys on the Pi and clear old keys from your client.

#### On the Raspberry Pi (Regenerate Host Keys):
```bash
# 1. Remove old or invalid SSH host keys
sudo rm -f /etc/ssh/ssh_host_*

# 2. Regenerate brand-new host keys for all cipher suites (RSA, ECDSA, ED25519)
sudo dpkg-reconfigure openssh-server

# 3. Restart the SSH daemon
sudo systemctl restart ssh
```

#### On your Client PC (Windows PowerShell / macOS / Linux Terminal):
If your computer throws a host key mismatch warning, clear the cached key for the Pi's IP:
```powershell
# Clear stale host key for your Pi's IP (e.g. 10.66.40.6 or 192.168.4.1)
ssh-keygen -R <PI_IP_ADDRESS>

# Example:
ssh-keygen -R 10.66.40.6

# Connect to the Raspberry Pi
ssh pi@<PI_IP_ADDRESS>
```

---

### 4.2 Raspberry Pi 5 System & Dependency Setup

#### Automated One-Command Setup:
```bash
cd ~/safety
chmod +x scripts/setup_pi.sh
./scripts/setup_pi.sh
```

#### Manual Setup Commands (Step-by-Step):
```bash
# 1. Update OS package lists
sudo apt update && sudo apt upgrade -y

# 2. Install essential system packages, Libcamera, OpenCV, and build tools
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

# 3. Grant user access to hardware peripherals (camera, GPIO, serial)
sudo usermod -aG video,dialout,gpio $USER

# 4. Create Python virtual environment with system site packages enabled
cd ~/safety
python3 -m venv --system-site-packages venv
source venv/bin/activate

# 5. Install PyTorch (ARM64 CPU-optimized, avoiding CUDA bloat) and Ultralytics
pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics
pip install -r requirements.txt
```

---

### 4.3 Network & Wi-Fi Configuration

```bash
# Find Raspberry Pi IP address on local network:
hostname -I

# Detailed wlan0 wireless interface status:
ip addr show wlan0

# Configure automated standalone Wi-Fi Access Point (SAFETY_POLE_AP):
chmod +x scripts/setup_hotspot.sh
sudo ./scripts/setup_hotspot.sh

# Or connect Raspberry Pi to a mobile hotspot / Wi-Fi network:
sudo nmcli dev wifi connect "<WIFI_SSID>" password "<WIFI_PASSWORD>"
```

---

### 4.4 Hardware Diagnostics & Camera Verification

Test and verify the **Raspberry Pi Camera Module Rev 1.3 (OV5647 CSI)** before launching the AI safety pipeline:

```bash
# 1. Check if the camera ribbon cable and OV5647 sensor are detected by libcamera:
rpicam-hello --list-cameras
# Expected output: 0 : ov5647 [2592x1944 10-bit GBRG]

# 2. Capture a test still image (verifies lens focus and exposure):
rpicam-still -t 2000 -o test_cam.jpg

# 3. Inspect video device nodes:
v4l2-ctl --list-devices
```

---

### 4.5 AI Model Accuracy, PPE & Latency Benchmarking

Use the built-in benchmarking tool [`scripts/test_model.py`](file:///c:/Users/s%20shane%20gilbert/OneDrive/Desktop/safety/scripts/test_model.py) to measure detection accuracy, PPE vest verification, and inference FPS:

```bash
# Benchmark Raspberry Pi 5 MIPI CSI Camera (OV5647):
source venv/bin/activate
python scripts/test_model.py --source picam2

# Benchmark USB Webcam (device index 0):
python scripts/test_model.py --source 0

# Benchmark synthetic scenario simulation:
python scripts/test_model.py --source synthetic

# Compare different YOLO models:
python scripts/test_model.py --model yolov8s.pt --source picam2
python scripts/test_model.py --model yolo11s.pt --source picam2
python scripts/test_model.py --model yolov8n.pt --source picam2
```

---

### 4.6 Running Automated Test Suites

Verify all 19 system units (IMU, LiDAR, BEV homography, EKF, Risk, CLAHE, PPE verification, and end-to-end pipeline):

```bash
source venv/bin/activate
python tests/run_all_tests.py
```

---

### 4.7 Launching the System

#### Mode 1: Physical Hardware Mode (Raspberry Pi 5 + Cameras + LiDAR + ESP32)
Starts all live sensors, opens UDP ports `5005` (Worker) and `5006` (LiDAR), enables hardware GPIO warning strobes, and serves the dashboard:
```bash
source venv/bin/activate
python main.py --mode hardware --port 8080
```

#### Mode 2: Auto Mode (Auto-detects Connected Hardware)
Probes for physical camera and serial devices; automatically initializes available sensors and falls back gracefully:
```bash
source venv/bin/activate
python main.py --mode auto
```

#### Mode 3: Interactive Simulation Mode (PC / Mac / Raspberry Pi)
Runs the interactive multi-agent simulator without requiring any physical sensors or ESP32 hardware:
```bash
# One-click simulation launcher:
python run_simulation.py

# Or via main orchestrator:
python main.py --mode simulation --port 8080
```

Open your browser to:
```text
http://localhost:8080
# Or from another device on the same network:
http://<PI_IP_ADDRESS>:8080
```

---

### 4.8 Running as a Background System Service (Auto-Start on Boot)

To run the Smart Work-Zone Safety Pole automatically whenever the Raspberry Pi powers on:

```bash
# 1. Copy the systemd unit file
sudo cp systemd/safety_pole.service /etc/systemd/system/

# 2. Reload systemd daemon
sudo systemctl daemon-reload

# 3. Enable service to start on system boot
sudo systemctl enable safety_pole.service

# 4. Start the service immediately
sudo systemctl start safety_pole.service

# 5. Check real-time service status:
sudo systemctl status safety_pole.service

# 6. View live streaming application logs:
journalctl -u safety_pole.service -f

# 7. Stop or restart the service:
sudo systemctl stop safety_pole.service
sudo systemctl restart safety_pole.service
```

---

## 5. ESP32 Firmware Flashing

### 5.1 Worker Safety Badge (`firmware/esp32_worker/`)
1. Open `firmware/esp32_worker/esp32_worker.ino` in Arduino IDE or VS Code PlatformIO.
2. Select Board: **ESP32S3 Dev Module**.
3. Edit `config.h` to set your WiFi SSID, Password, and Raspberry Pi IP address (`192.168.4.1` or your network IP).
4. Wire MPU6500: `SDA -> GPIO 8`, `SCL -> GPIO 9`, `VCC -> 3.3V`, `GND -> GND`.
5. Upload sketch to the ESP32-S3.

### 5.2 LiDAR Bridge (`firmware/esp32_lidar/`)
1. Open `firmware/esp32_lidar/esp32_lidar.ino`.
2. Wire TSD20 LiDAR: `TX -> GPIO 16 (RX2)`, `RX -> GPIO 17 (TX2)`, `5V -> 5V`, `GND -> GND`.
3. Set WiFi credentials in `config.h` and upload sketch.

