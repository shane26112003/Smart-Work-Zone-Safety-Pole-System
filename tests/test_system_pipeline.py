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
    cam_mgr = CameraManager()
    alert_ctrl = AlertController()
    worker_rx = WorkerReceiver(port=5996)
    lidar = LidarTSD20(mode="mock")

    try:
        # 1. Perception & Fusion Engine initialization
        cam_mgr.start()
        detector = ObjectDetector()
        tracker = MultiObjectTracker()
        bev = BEVTransform()
        fusion = SensorFusionEngine(bev)
        risk_engine = RiskEngine()
        logger = EventLogger()

        # 2. Inject fresh worker state and LiDAR reading
        worker_rx.inject_worker_state(WorkerState(
            worker_id="WORKER_01",
            motion_state="STATIC",
            x=-3.0, y=4.0,
            last_seen=time.time()
        ))
        lidar.inject_reading(distance_m=15.0, range_rate_mps=-10.0, valid=True)

        # 3. Ingest camera frame
        time.sleep(0.2) # Allow frame capture
        road_frame = cam_mgr.get_road_frame()
        assert road_frame is not None

        # 4. Detect & Track
        detections = detector.detect(road_frame)
        tracks = tracker.update(detections)

        # 5. Fuse Multi-Sensor Inputs
        reading = lidar.get_latest_reading()
        workers = worker_rx.get_all_active_workers()
        entities = fusion.fuse(tracks, reading, workers)
        assert len(entities) >= 1

        # 6. Evaluate Risk
        assessment = risk_engine.evaluate(entities)
        assert assessment.overall_level in ["SAFE", "CAUTION", "HIGH RISK", "CRITICAL"]

        # 7. Actuator Update & Event Log
        alert_ctrl.update(assessment)
        logger.log_assessment(assessment)

    finally:
        # Guarantee clean cleanup of all background threads
        cam_mgr.stop()
        alert_ctrl.stop()
        worker_rx.stop()
        lidar.stop()
