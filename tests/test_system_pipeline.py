"""
End-to-End System Pipeline Integration Test.
Verifies complete flow from sensor ingestion through fusion, risk, and alert outputs.
"""

import time
from src.sensors.camera_manager import CameraManager
from src.sensors.lidar_tsd20 import LidarTSD20
from src.sensors.worker_receiver import WorkerReceiver, WorkerState
from src.cv.detector import ObjectDetector
from src.cv.tracker import MultiObjectTracker
from src.fusion.bev_transform import BEVTransform
from src.fusion.ekf_fusion import SensorFusionEngine
from src.risk.risk_engine import RiskEngine
from src.warning.alert_controller import AlertController
from src.warning.event_logger import EventLogger

def test_full_pipeline_step():
    """Executes a complete perception-to-warning iteration."""
    # 1. Sensors
    cam_mgr = CameraManager()
    cam_mgr.start()
    lidar = LidarTSD20(mode="mock")
    worker_rx = WorkerReceiver(port=5996)

    # Inject worker state
    worker_rx.inject_worker_state(WorkerState(
        worker_id="WORKER_01",
        motion_state="STATIC",
        x=-3.0, y=4.0
    ))

    # Inject LiDAR reading
    lidar.inject_reading(distance_m=15.0, range_rate_mps=-10.0, valid=True)

    # 2. Perception & Fusion
    detector = ObjectDetector()
    tracker = MultiObjectTracker()
    bev = BEVTransform()
    fusion = SensorFusionEngine(bev)
    risk_engine = RiskEngine()
    alert_ctrl = AlertController()
    logger = EventLogger()

    time.sleep(0.15) # Allow camera frame buffer to warm up
    road_frame = cam_mgr.get_road_frame()
    assert road_frame is not None

    # Detect
    detections = detector.detect(road_frame)
    tracks = tracker.update(detections)

    # Fuse
    reading = lidar.get_latest_reading()
    workers = worker_rx.get_all_active_workers()
    entities = fusion.fuse(tracks, reading, workers)
    assert len(entities) >= 1

    # Evaluate Risk
    assessment = risk_engine.evaluate(entities)
    assert assessment.overall_level in ["SAFE", "CAUTION", "HIGH RISK", "CRITICAL"]

    # Actuator Update & Event Log
    alert_ctrl.update(assessment)
    logger.log_assessment(assessment)

    # Cleanup
    cam_mgr.stop()
    alert_ctrl.stop()
