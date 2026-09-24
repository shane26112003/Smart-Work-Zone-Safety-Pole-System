"""
Smart Work-Zone Safety Pole System - Global Configuration
Centralized configuration parameters for sensors, CV, fusion, risk assessment, and UI.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Any
import os

@dataclass
class NetworkConfig:
    """Network listener ports for wireless sensor ingestion."""
    HOST: str = "0.0.0.0"
    WORKER_UDP_PORT: int = 5005      # Port for ESP32-S3 Worker telemetry packets
    LIDAR_UDP_PORT: int = 5006       # Port for ESP32 LiDAR distance packets
    DASHBOARD_PORT: int = 8080       # Port for Tornado Web Dashboard & WebSockets


@dataclass
class LidarConfig:
    """TSD20 LiDAR sensor configuration."""
    SOURCE_TYPE: str = "wifi"        # "wifi" (via ESP32), "serial" (direct UART/USB), or "mock"
    SERIAL_PORT: str = "/dev/ttyAMA0" # Fallback if direct UART on Pi 5 (or 'COM3' on Windows)
    BAUD_RATE: int = 115200
    MIN_RANGE_M: float = 0.10        # Minimum reliable distance
    MAX_RANGE_M: float = 20.0        # Max range for TSD20 (meters)
    FOV_DEG: float = 2.0             # Beam divergence / field of view
    ORIENTATION_DEG: float = 0.0     # Aimed directly along road approach line (0 deg = road center)
    RATE_HZ: int = 50                # Expected sample frequency


@dataclass
class WorkerTagConfig:
    """Worker Module (ESP32-S3 + MPU6500) settings."""
    DEFAULT_WORKER_ID: str = "WORKER_01"
    HEARTBEAT_TIMEOUT_SEC: float = 5.0   # Drop worker track if no telemetry for 5s
    BLE_TX_POWER_1M: float = -59.0       # Calibrated RSSI at 1 meter (A parameter)
    BLE_PATH_LOSS_EXPONENT: float = 2.4  # Environmental path loss exponent (n parameter)
    IMU_FALL_SVM_THRESHOLD: float = 3.0  # Impact G-force threshold for fall detection
    IMU_FREEFALL_SVM_THRESHOLD: float = 0.4 # Freefall low-G threshold before impact


@dataclass
class CameraConfig:
    """Dual camera configurations (Road-facing & Work-zone facing)."""
    ROAD_CAM_ID: Any = "picam2"       # Bound to Picamera2 for Raspberry Pi CSI
    WORKZONE_CAM_ID: Any = "mock"     # Synthetic workzone feed (single physical camera setup)
    FRAME_WIDTH: int = 640
    FRAME_HEIGHT: int = 480
    TARGET_FPS: int = 25
    USE_PICAMERA2: bool = True        # Enabled for Raspberry Pi 5 MIPI CSI (OV5647 Rev 1.3)


@dataclass
class CVConfig:
    """Computer Vision & Tracking configuration."""
    # Model configuration: yolov8s.pt (high accuracy, 44.9 mAP) with automated fallbacks (yolo11s/yolov8n)
    MODEL_PATH: str = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "yolov8s.pt")
    FALLBACK_MODEL_PATH: str = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "yolov8n.pt")
    CONFIDENCE_THRESHOLD: float = 0.25    # Base threshold
    IOU_THRESHOLD: float = 0.45
    IMAGE_SIZE: int = 640                 # Inference resolution (matching camera width)

    # Class-Specific Confidence Thresholds (prevents small object noise from triggering false vehicle detections)
    WORKER_CONF_THRESHOLD: float = 0.28   # Sensitive threshold for workers (standing, bending, walking)
    VEHICLE_CONF_THRESHOLD: float = 0.45  # Higher threshold for vehicles to eliminate false detections on small specks
    BICYCLE_CONF_THRESHOLD: float = 0.50  # Strict threshold if bicycles are present

    # Minimum Geometric Size Filters (prevents small background objects from being classified as vehicles)
    VEHICLE_MIN_WIDTH_PX: int = 30        # Vehicle must be at least 30 px wide
    VEHICLE_MIN_HEIGHT_PX: int = 24       # Vehicle must be at least 24 px high
    VEHICLE_MIN_AREA_PX: int = 750        # Minimum pixel area for a valid vehicle (eliminates small debris/objects)
    VEHICLE_MIN_ASPECT_RATIO: float = 0.55 # Width / Height (vehicles are horizontal)
    VEHICLE_MAX_ASPECT_RATIO: float = 4.0  # Width / Height upper bound

    # Worker Geometric Sanity Filters
    WORKER_MIN_HEIGHT_PX: int = 35        # A person must be at least 35 px tall
    WORKER_MIN_ASPECT_RATIO: float = 0.70 # Height / Width (people are upright)

    # Image enhancement (Adaptive CLAHE only for extremely dark or washed-out lighting)
    ENABLE_ENHANCEMENT: bool = False       # Disabled by default to prevent sensor noise amplification
    ADAPTIVE_CLAHE: bool = True           # Automatically activates only if frame dynamic range < 18
    CLAHE_CLIP_LIMIT: float = 1.5

    # Intelligent Worker PPE Verification (High-Vis Vest & Hard Hat detection)
    ENABLE_PPE_CHECK: bool = True
    PPE_MIN_RATIO: float = 0.05           # Minimum high-vis vest color ratio in upper body
    STRICT_PPE_MODE: bool = False         # Keep False so all detected people are identified as WORKERS

    # COCO Class IDs: 0=person, 2=car, 3=motorcycle, 5=bus, 7=truck (Class 1 bicycle optional)
    WORKER_CLASSES: List[int] = field(default_factory=lambda: [0])
    VEHICLE_CLASSES: List[int] = field(default_factory=lambda: [2, 3, 5, 7])
    BICYCLE_CLASSES: List[int] = field(default_factory=lambda: [1])

    # Tracking parameters
    TRACK_MAX_AGE_FRAMES: int = 30
    TRACK_MIN_HITS: int = 2               # 2 hits for fast track initiation without lag


@dataclass
class GeometryConfig:
    """
    Work-zone spatial coordinate system (in meters relative to Safety Pole at (0, 0)).
    X-axis: Lateral across road & work zone (Negative = Work Zone, Positive = Road Lanes)
    Y-axis: Longitudinal along road (0 = at pole, Positive = Approaching traffic direction)
    """
    SAFETY_POLE_POS: Tuple[float, float] = (0.0, 0.0)
    WORKZONE_X_MIN: float = -12.0
    WORKZONE_X_MAX: float = -0.5    # 0.5m buffer from pole
    HAZARD_LINE_X: float = 0.0      # Safety cone divider line between work zone and road
    ROAD_LANE_X_MIN: float = 0.0
    ROAD_LANE_X_MAX: float = 8.0    # 2 traffic lanes (4m each)
    DETECTION_RANGE_Y_MAX: float = 60.0
    LIDAR_MOUNT_OFFSET: Tuple[float, float] = (0.0, 0.0) # Mounted directly on pole


@dataclass
class RiskConfig:
    """Collision-Risk Assessment Engine thresholds."""
    # Time-To-Collision (TTC) thresholds in seconds
    TTC_CRITICAL_SEC: float = 2.5
    TTC_HIGH_RISK_SEC: float = 4.5
    TTC_CAUTION_SEC: float = 7.0

    # Distance thresholds in meters
    DIST_CRITICAL_M: float = 2.5
    DIST_HIGH_RISK_M: float = 6.0
    DIST_CAUTION_M: float = 12.0

    # Lateral proximity threshold to boundary line
    HAZARD_LINE_BUFFER_M: float = 1.0  # If worker within 1.0m of road boundary

    # Closest Point of Approach (CPA) miss-distance threshold
    CPA_COLLISION_DISTANCE_M: float = 2.0


@dataclass
class WarningConfig:
    """Warning actuator settings (GPIO / siren / visual)."""
    USE_REAL_GPIO: bool = False       # True on Pi 5 with RPi.GPIO or gpiod
    PIN_CAUTION_LED: int = 22        # Amber directional LED flasher
    PIN_DANGER_LED: int = 17         # High-intensity Red strobe
    PIN_SIREN_RELAY: int = 27        # 12V Smart siren relay
    PIN_BUZZER: int = 23             # Directional audio buzzer
    AUDIO_ALARM_ENABLED: bool = True  # Dashboard browser audio chime


@dataclass
class SystemConfig:
    """Master configuration aggregator."""
    network: NetworkConfig = field(default_factory=NetworkConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    worker: WorkerTagConfig = field(default_factory=WorkerTagConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    cv: CVConfig = field(default_factory=CVConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    risk: RiskConfig = field(default_factory=RiskConfig)
    warning: WarningConfig = field(default_factory=WarningConfig)

    # Logging
    LOG_DIR: str = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
    INCIDENT_LOG_FILE: str = "incident_log.jsonl"


# Global singleton configuration instance
CONFIG = SystemConfig()
