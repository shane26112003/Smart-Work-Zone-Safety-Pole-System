"""
Smart Work-Zone Safety Pole System - Primary Application Orchestrator.
Fuses AI Computer Vision, TSD20 LiDAR, ESP32-S3 Worker Telemetry,
EKF State Estimation, and Real-Time Risk Assessment.
"""

import sys
import time
import signal
import argparse
import logging
from typing import Any, Optional
import cv2
import numpy as np

from src.config import CONFIG
from src.sensors.lidar_tsd20 import LidarTSD20
from src.sensors.worker_receiver import WorkerReceiver
from src.sensors.camera_manager import CameraManager
from src.cv.detector import ObjectDetector
from src.cv.tracker import MultiObjectTracker
from src.fusion.bev_transform import BEVTransform
from src.fusion.ekf_fusion import SensorFusionEngine
from src.risk.risk_engine import RiskEngine
from src.warning.alert_controller import AlertController
from src.warning.event_logger import EventLogger
from src.dashboard.server import DashboardServer
from simulation.scenario_simulator import ScenarioSimulator

# Configure rich logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] (%(name)s) %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("SafetyPole")


class SafetyPoleSystem:
    """Central manager orchestrating all subsystems."""

    def __init__(self, mode: str = "auto", camera_source: Any = None, model_path: Optional[str] = None):
        self.mode = mode
        self.running = False

        logger.info("Initializing Smart Work-Zone Safety Pole System...")

        if camera_source is not None:
            if str(camera_source).lower() == "picam2":
                CONFIG.camera.ROAD_CAM_ID = "picam2"
                CONFIG.camera.USE_PICAMERA2 = True
            elif str(camera_source).isdigit():
                CONFIG.camera.ROAD_CAM_ID = int(camera_source)
                CONFIG.camera.USE_PICAMERA2 = False
            elif str(camera_source).lower() in ["mock", "synthetic", "sim"]:
                CONFIG.camera.ROAD_CAM_ID = "mock"
                CONFIG.camera.USE_PICAMERA2 = False
            else:
                CONFIG.camera.ROAD_CAM_ID = camera_source
                CONFIG.camera.USE_PICAMERA2 = False

        if model_path:
            CONFIG.cv.MODEL_PATH = model_path

        # 1. Sensor Ingestion Layer
        lidar_src = "mock" if mode == "simulation" else CONFIG.lidar.SOURCE_TYPE
        self.lidar = LidarTSD20(mode=lidar_src)
        self.worker_rx = WorkerReceiver()
        self.camera_mgr = CameraManager(road_cam_id=CONFIG.camera.ROAD_CAM_ID)

        # 2. Perception & AI Vision Layer
        self.detector = ObjectDetector(model_path=CONFIG.cv.MODEL_PATH)
        self.tracker = MultiObjectTracker(max_age=CONFIG.cv.TRACK_MAX_AGE_FRAMES)

        # 3. Fusion & Calibration Layer
        self.bev = BEVTransform(CONFIG.camera.FRAME_WIDTH, CONFIG.camera.FRAME_HEIGHT)
        self.fusion = SensorFusionEngine(self.bev)

        # 4. Risk Assessment & Safety Engine
        self.risk_engine = RiskEngine()

        # 5. Warning Actuators & Incident Logger
        self.alert_ctrl = AlertController()
        self.logger_db = EventLogger()

        # 6. Real-Time Web Operations Dashboard
        self.dashboard = DashboardServer(port=CONFIG.network.DASHBOARD_PORT)

        # 7. Simulator (if requested or in mock mode)
        self.simulator = None
        if mode in ["simulation", "auto"]:
            self.simulator = ScenarioSimulator(self.lidar, self.worker_rx, self.camera_mgr)

    def start(self):
        """Starts all subsystems and enters processing loop."""
        self.running = True

        # Start components
        self.lidar.start()
        self.worker_rx.start()
        self.camera_mgr.start()
        self.alert_ctrl.start()

        # Connect scenario switch callback
        scen_cb = self.simulator.set_scenario if self.simulator else None
        self.dashboard.start(scenario_callback=scen_cb)

        if self.simulator and self.mode == "simulation":
            self.simulator.start()

        logger.info("=" * 60)
        logger.info("SAFETY POLE SYSTEM OPERATIONAL")
        logger.info(f"Dashboard available at: http://localhost:{CONFIG.network.DASHBOARD_PORT}")
        logger.info("Press Ctrl+C to stop.")
        logger.info("=" * 60)

        self._main_loop()

    def stop(self):
        """Clean shutdown of all subsystems."""
        if not self.running:
            return
        logger.info("Shutting down Safety Pole System...")
        self.running = False

        if self.simulator:
            self.simulator.stop()
        self.dashboard.stop()
        self.alert_ctrl.stop()
        self.camera_mgr.stop()
        self.worker_rx.stop()
        self.lidar.stop()
        logger.info("All subsystems cleanly terminated.")

    def _main_loop(self):
        fps_target = CONFIG.camera.TARGET_FPS
        frame_interval = 1.0 / fps_target

        while self.running:
            loop_start = time.time()

            try:
                # 1. Ingest Camera Frames
                road_frame = self.camera_mgr.get_road_frame()
                workzone_frame = self.camera_mgr.get_workzone_frame()

                # 2. Ingest LiDAR & Worker Telemetry
                lidar_reading = self.lidar.get_latest_reading()
                active_workers = self.worker_rx.get_all_active_workers()

                # 3. Computer Vision: Object Detection on Road Camera
                detections = []
                if road_frame is not None:
                    is_synth = self.camera_mgr.is_road_synthetic()
                    detections = self.detector.detect(road_frame, is_synthetic=is_synth)

                # 4. Multi-Object Tracking
                tracks = self.tracker.update(detections)

                # 5. Multi-Sensor EKF Fusion (Camera + LiDAR + ESP32 Worker)
                fused_entities = self.fusion.fuse(tracks, lidar_reading, active_workers)

                # 6. Collision-Risk Assessment
                assessment = self.risk_engine.evaluate(fused_entities)

                # 7. Actuator Alerts
                self.alert_ctrl.update(assessment)

                # Dispatch targeted evasive guidance directly to specific worker at risk
                if assessment.guidance and assessment.overall_level in ["CAUTION", "HIGH RISK", "CRITICAL"]:
                    self.worker_rx.send_evasive_guidance(assessment.guidance)

                # 8. Event Logging
                self.logger_db.log_assessment(assessment)

                # 9. Annotate Camera Frames with Tactical Overlays
                annotated_road = self._annotate_frame(road_frame, tracks, assessment, lidar_reading)

                # 10. Broadcast Telemetry to Web Dashboard
                actuators_state = {
                    "caution_led": self.alert_ctrl.caution_led_on,
                    "danger_led": self.alert_ctrl.danger_led_on,
                    "siren": self.alert_ctrl.siren_active,
                    "buzzer": self.alert_ctrl.buzzer_active
                }

                self.dashboard.update_telemetry(
                    assessment=assessment,
                    entities=fused_entities,
                    lidar=lidar_reading,
                    workers=active_workers,
                    road_frame=annotated_road,
                    workzone_frame=workzone_frame,
                    actuators=actuators_state
                )

            except Exception as e:
                logger.error(f"Error in main processing loop: {e}", exc_info=True)

            elapsed = time.time() - loop_start
            time.sleep(max(0.002, frame_interval - elapsed))

    def _annotate_frame(self, frame: np.ndarray, tracks: list, assessment, lidar) -> np.ndarray:
        """Draws tactical bounding boxes, IDs, speeds, and threat banners on video feed."""
        if frame is None:
            return None
        out = frame.copy()
        h, w = out.shape[:2]

        # Draw risk banner along top
        banner_colors = {
            "SAFE": (16, 185, 129),
            "CAUTION": (245, 158, 11),
            "HIGH RISK": (249, 115, 22),
            "CRITICAL": (239, 68, 68)
        }
        b_color_rgb = banner_colors.get(assessment.overall_level, (100, 100, 100))
        b_color = (b_color_rgb[2], b_color_rgb[1], b_color_rgb[0]) # BGR

        cv2.rectangle(out, (0, 0), (w, 36), b_color, -1)
        active_model = getattr(self.detector, "active_model_name", "AI")
        cv2.putText(out, f"RISK: {assessment.overall_level} | TTC: {assessment.min_ttc_sec:.1f}s | DIST: {assessment.min_distance_m:.1f}m | MODEL: {active_model}", 
                    (15, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        # Draw track boxes with tactical styling
        for trk in tracks:
            x1, y1, x2, y2 = trk.bbox
            is_veh = (trk.class_name == "vehicle")
            conf_pct = int(trk.confidence * 100)

            if is_veh:
                color = (0, 140, 255) # High-visibility Amber/Orange for vehicles
                sub = getattr(trk, "subclass", "")
                label = f"{sub.upper() or 'VEHICLE'} #{trk.track_id} ({conf_pct}%)"
            else:
                # Always identify detected humans as WORKERS
                if trk.associated_worker_id:
                    color = (0, 255, 0)
                    label = f"👷 {trk.associated_worker_id} ({conf_pct}%)"
                elif getattr(trk, "ppe_verified", False):
                    color = (0, 255, 0) # Bright Green for verified PPE
                    label = f"👷 WORKER #{trk.track_id} [PPE ✓] ({conf_pct}%)"
                else:
                    color = (50, 205, 50) # Tactical Lime Green
                    label = f"👷 WORKER #{trk.track_id} ({conf_pct}%)"

            # Bounding box
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

            lbl_w = max(90, len(label) * 9 + 10)
            cv2.rectangle(out, (x1, max(0, y1 - 22)), (x1 + lbl_w, y1), color, -1)
            cv2.putText(out, label, (x1 + 4, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)

            # Ground contact dot
            cv2.circle(out, trk.ground_point, 4, (0, 0, 255), -1)

        return out


def main():
    parser = argparse.ArgumentParser(description="Smart Work-Zone Safety Pole System")
    parser.add_argument("--mode", choices=["auto", "simulation", "hardware"], default="auto",
                        help="Execution mode (simulation=synthetic actors, hardware=physical devices)")
    parser.add_argument("--port", type=int, default=CONFIG.network.DASHBOARD_PORT,
                        help="Web Dashboard port")
    parser.add_argument("--camera-source", "--camera", "--source", dest="camera_source", type=str, default=None,
                        help="Road camera source: 'picam2', camera index (e.g. '0'), 'synthetic', or video path")
    parser.add_argument("--model", type=str, default=None,
                        help="Path to custom YOLO model weights (.pt file)")
    parser.add_argument("--lidar-source", dest="lidar_source", type=str, default=None,
                        help="LiDAR input source: serial port or 'mock'")
    args = parser.parse_args()

    if args.port:
        CONFIG.network.DASHBOARD_PORT = args.port
    if args.lidar_source:
        CONFIG.lidar.SOURCE_TYPE = args.lidar_source

    system = SafetyPoleSystem(mode=args.mode, camera_source=args.camera_source, model_path=args.model)

    # Signal handlers for graceful exit
    def sig_handler(sig, frame):
        system.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    system.start()


if __name__ == "__main__":
    main()
