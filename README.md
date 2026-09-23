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
- **Detector**: Lightweight YOLOv8 (`yolov8n.pt`) optimized for Raspberry Pi 5 CPU/GPU.
- **Tracker**: Multi-Object Tracking with Kalman Filter and Hungarian IoU association, computing object velocity vectors and maintaining persistent IDs.

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
├── tests/
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

## 4. Quickstart Guide

### 4.1 Running in Interactive Simulation Mode (PC or Pi 5)
You can run the full system immediately without any physical hardware connected:

```bash
python run_simulation.py
```

Then open your browser to:
```
http://localhost:8080
```

Inside the dashboard:
- View live dual camera streams (road camera with YOLO detection overlays and work-zone camera).
- Watch the tactical 2D Bird's-Eye View (BEV) radar showing vehicle trajectories, workers, and LiDAR ranging beams.
- Click the **Interactive Scenario Testing** buttons to test:
  1. **Normal Passing Traffic** (Safe separation)
  2. **Distracted Driver** (Vehicle drifts across safety cones towards worker $\to$ triggers `CRITICAL` siren and strobes)
  3. **Worker Incursion** (Worker steps across boundary line into active traffic)
  4. **Worker Down** (ESP32 reports `FALL_DETECTED` $\to$ triggers emergency alert)

---

### 4.2 Running on Raspberry Pi 5 with Physical Hardware

1. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```
2. **Start the System**:
   ```bash
   python main.py --mode hardware
   ```
   The system will bind UDP ports `5005` (Worker) and `5006` (LiDAR), initialize cameras and GPIO pins, and start the web dashboard at port `8080`.

---

## 5. ESP32 Firmware Flashing

### 5.1 Worker Safety Badge (`firmware/esp32_worker/`)
1. Open `firmware/esp32_worker/esp32_worker.ino` in Arduino IDE or VS Code PlatformIO.
2. Select Board: **ESP32S3 Dev Module**.
3. Edit `config.h` to set your WiFi SSID, Password, and Raspberry Pi IP address (`192.168.4.1`).
4. Wire MPU6500: `SDA -> GPIO 8`, `SCL -> GPIO 9`, `VCC -> 3.3V`, `GND -> GND`.
5. Upload to the ESP32-S3.

### 5.2 LiDAR Bridge (`firmware/esp32_lidar/`)
1. Open `firmware/esp32_lidar/esp32_lidar.ino`.
2. Wire TSD20 LiDAR: `TX -> GPIO 16 (RX2)`, `RX -> GPIO 17 (TX2)`, `5V -> 5V`, `GND -> GND`.
3. Set WiFi credentials in `config.h` and upload.

---

## 6. Running Tests

Run the complete automated test suite verifying all modules:

```bash
python tests/run_all_tests.py
```

All 12 test suites will run and report:
- IMU feature calculations and fall detection state machine
- TSD20 LiDAR parsing and range rate filtering
- BEV homography consistency and EKF state updates
- Collision risk calculations (TTC, CPA, and vulnerability weighting)
- Full end-to-end perception-to-warning pipeline
