"""
Unit tests for BEV homography coordinate transformation and EKF sensor fusion.
"""

import math
from src.fusion.bev_transform import BEVTransform
from src.fusion.ekf_fusion import SensorFusionEngine, EKFStateFilter
from src.cv.tracker import TrackedObject
from src.sensors.lidar_tsd20 import LidarReading
from src.sensors.worker_receiver import WorkerState

def test_bev_transform_consistency():
    """Verify forward and inverse perspective mapping."""
    bev = BEVTransform(image_width=640, image_height=480)

    # Road center point
    u, v = 320, 360
    xm, ym = bev.image_to_metric(u, v)
    assert ym >= 0.0 # Distance along road must be positive

    # Project back
    u_back, v_back = bev.metric_to_image(xm, ym)
    assert abs(u - u_back) < 5
    assert abs(v - v_back) < 5

def test_ekf_filter_prediction_and_update():
    """Verify Kalman filter prediction and LiDAR correction."""
    kf = EKFStateFilter(x0=3.0, y0=25.0, vx0=0.0, vy0=-10.0, is_vehicle=True)
    
    # Predict forward 0.1s
    kf.predict(dt=0.1)
    # y should decrease because vy is negative
    assert kf.state[1, 0] < 25.0

    # Camera measurement update
    kf.update_camera(x_cam=3.1, y_cam=24.0)
    assert abs(kf.state[0, 0] - 3.1) < 0.5

    # High precision LiDAR measurement update
    kf.update_lidar(dist_m=23.95, range_rate_mps=-10.0)
    assert abs(kf.state[1, 0] - 23.95) < 0.3

def test_sensor_fusion_engine_association():
    """Verify associating visual track with ESP32 worker state."""
    fusion = SensorFusionEngine()
    
    # Mock camera track for a worker
    wrk_track = TrackedObject(
        track_id=1,
        class_name="worker",
        bbox=(200, 300, 240, 380),
        centroid=(220, 340),
        ground_point=(220, 380)
    )

    lidar = LidarReading(distance_m=20.0, valid=False)
    
    worker_state = WorkerState(
        worker_id="WORKER_01",
        motion_state="WALKING",
        svm=1.2,
        estimated_dist_m=3.0,
        x=-3.0, y=2.0
    )

    entities = fusion.fuse([wrk_track], lidar, [worker_state])
    assert len(entities) >= 1
    
    worker_entity = next((e for e in entities if e.entity_type == "worker"), None)
    assert worker_entity is not None
    assert worker_entity.associated_worker_id == "WORKER_01"
    assert worker_entity.worker_state.motion_state == "WALKING"
