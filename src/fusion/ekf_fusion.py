"""
Multi-Sensor Extended Kalman Filter (EKF) & Work-Zone Fusion Engine.
Fuses:
  1. Camera Vision Tracks (BEV Position & Velocity)
  2. TSD20 LiDAR (1D Precision Distance & Range-Rate)
  3. Worker ESP32 Wearable Tag (Worker ID, MPU6500 IMU dynamics, BLE RSSI proximity)
Creates a unified, calibrated common operational picture of the work zone.
"""

import time
import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import logging

from ..config import CONFIG
from .bev_transform import BEVTransform
from ..cv.tracker import TrackedObject
from ..sensors.lidar_tsd20 import LidarReading
from ..sensors.worker_receiver import WorkerState

logger = logging.getLogger("SensorFusion")

@dataclass
class FusedEntity:
    """Represents a unified, multi-sensor fused object in metric coordinates."""
    entity_id: str                      # "VEH_01", "WORKER_01", etc.
    entity_type: str                    # "vehicle" or "worker"
    x: float                            # Lateral position in meters (rel to pole)
    y: float                            # Longitudinal position in meters along road
    vx: float                           # Lateral velocity (m/s)
    vy: float                           # Longitudinal velocity (m/s)
    speed_mps: float = 0.0              # Scalar speed in m/s
    speed_kmh: float = 0.0              # Speed in km/h
    heading_deg: float = 0.0            # Direction of travel (degrees)
    predicted_trajectory: List[Tuple[float, float]] = field(default_factory=list) # 3-second horizon
    confidence: float = 0.8
    # Sensor provenance flags
    camera_track_id: Optional[int] = None
    associated_worker_id: Optional[str] = None
    worker_state: Optional[WorkerState] = None
    lidar_verified: bool = False
    lidar_distance_m: Optional[float] = None
    last_update: float = field(default_factory=time.time)


class EKFStateFilter:
    """Extended Kalman Filter for individual vehicle or worker state [x, y, vx, vy]."""

    def __init__(self, x0: float, y0: float, vx0: float = 0.0, vy0: float = 0.0, is_vehicle: bool = True):
        self.state = np.array([[x0], [y0], [vx0], [vy0]], dtype=float)
        # Covariance matrix P
        self.P = np.diag([2.0, 2.0, 5.0, 5.0])
        
        # Process noise Q
        q_pos = 0.05 if is_vehicle else 0.02
        q_vel = 0.20 if is_vehicle else 0.10
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel])
        
        # Camera measurement noise R_cam (meters)
        self.R_cam = np.diag([0.6, 1.2]) # Higher longitudinal error from camera perspective
        # LiDAR measurement noise R_lidar (meters & m/s)
        self.R_lidar = np.diag([0.05, 0.2])

    def predict(self, dt: float):
        """Predict state forward by dt seconds."""
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.state = np.dot(F, self.state)
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q

    def update_camera(self, x_cam: float, y_cam: float):
        """Measurement update from Camera BEV."""
        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])
        z = np.array([[x_cam], [y_cam]])
        y = z - np.dot(H, self.state)
        S = np.dot(np.dot(H, self.P), H.T) + self.R_cam
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        self.state = self.state + np.dot(K, y)
        I = np.eye(4)
        self.P = np.dot(I - np.dot(K, H), self.P)

    def update_lidar(self, dist_m: float, range_rate_mps: float):
        """
        EKF update from TSD20 LiDAR.
        Measurement model: r = sqrt(x^2 + y^2), r_dot = (x*vx + y*vy) / r
        """
        x = self.state[0, 0]
        y = self.state[1, 0]
        vx = self.state[2, 0]
        vy = self.state[3, 0]
        
        r = math.sqrt(x*x + y*y)
        if r < 0.1:
            return

        r_dot = (x*vx + y*vy) / r
        z = np.array([[dist_m], [range_rate_mps]])
        h_x = np.array([[r], [r_dot]])
        
        # Jacobian H matrix
        H = np.zeros((2, 4))
        H[0, 0] = x / r
        H[0, 1] = y / r
        H[1, 0] = (vx * r - (x*vx + y*vy) * (x/r)) / (r*r)
        H[1, 1] = (vy * r - (x*vx + y*vy) * (y/r)) / (r*r)
        H[1, 2] = x / r
        H[1, 3] = y / r

        y_res = z - h_x
        S = np.dot(np.dot(H, self.P), H.T) + self.R_lidar
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        self.state = self.state + np.dot(K, y_res)
        I = np.eye(4)
        self.P = np.dot(I - np.dot(K, H), self.P)


class SensorFusionEngine:
    """
    Coordinates multi-sensor fusion across Camera, LiDAR, and ESP32 Worker telemetry.
    Produces high-fidelity FusedEntity objects with future trajectory rollouts.
    """

    def __init__(self, bev_transform: Optional[BEVTransform] = None):
        self.bev = bev_transform or BEVTransform()
        self.filters: Dict[str, EKFStateFilter] = {}
        self.last_timestamp = time.time()

    def fuse(self, 
             camera_tracks: List[TrackedObject], 
             lidar_reading: LidarReading, 
             worker_states: List[WorkerState]) -> List[FusedEntity]:
        """
        Fuses current sensor inputs into a coherent work-zone state.
        """
        now = time.time()
        dt = max(0.01, min(0.2, now - self.last_timestamp))
        self.last_timestamp = now

        fused_entities: List[FusedEntity] = []

        # 1. Process Visual Tracks (Vehicles & Workers)
        for trk in camera_tracks:
            # Map image ground point to metric BEV (X, Y)
            gx, gy = trk.ground_point
            x_m, y_m = self.bev.image_to_metric(gx, gy)
            is_vehicle = (trk.class_name == "vehicle")
            entity_id = f"VEH_{trk.track_id:02d}" if is_vehicle else f"WRK_CAM_{trk.track_id:02d}"

            # EKF State instance
            if entity_id not in self.filters:
                # Estimate initial velocity
                self.filters[entity_id] = EKFStateFilter(x_m, y_m, 0.0, -10.0 if is_vehicle else 0.0, is_vehicle)

            kf = self.filters[entity_id]
            kf.predict(dt)
            kf.update_camera(x_m, y_m)

            # Check LiDAR gating for approaching vehicles
            lidar_matched = False
            lidar_dist = None
            if is_vehicle and lidar_reading.valid and (0.5 <= lidar_reading.distance_m <= 20.0):
                # Gate: if vehicle is in road lane and longitudinal distance aligns within gate
                pred_y = kf.state[1, 0]
                if abs(pred_y - lidar_reading.distance_m) < 4.5:
                    kf.update_lidar(lidar_reading.distance_m, lidar_reading.range_rate_mps)
                    lidar_matched = True
                    lidar_dist = lidar_reading.distance_m

            # Extract updated state
            fx = float(kf.state[0, 0])
            fy = float(kf.state[1, 0])
            fvx = float(kf.state[2, 0])
            fvy = float(kf.state[3, 0])
            speed = math.sqrt(fvx*fvx + fvy*fvy)
            heading = math.degrees(math.atan2(fvx, -fvy)) # 0 deg is straight down road

            # Trajectory Prediction (rollout for 3.0 seconds @ 0.5s steps)
            trajectory = []
            for step in range(1, 7):
                horizon_t = step * 0.5
                px = fx + fvx * horizon_t
                py = fy + fvy * horizon_t
                trajectory.append((round(px, 2), round(py, 2)))

            fused = FusedEntity(
                entity_id=entity_id,
                entity_type=trk.class_name,
                x=round(fx, 2),
                y=round(fy, 2),
                vx=round(fvx, 2),
                vy=round(fvy, 2),
                speed_mps=round(speed, 2),
                speed_kmh=round(speed * 3.6, 1),
                heading_deg=round(heading, 1),
                predicted_trajectory=trajectory,
                confidence=trk.confidence,
                camera_track_id=trk.track_id,
                lidar_verified=lidar_matched,
                lidar_distance_m=lidar_dist,
                last_update=now
            )
            fused_entities.append(fused)

        # 2. Worker Association: Pair detected person tracks with active ESP32 tags
        assigned_worker_ids = set()
        for entity in fused_entities:
            if entity.entity_type == "worker":
                # Find closest ESP32 worker state based on distance / RSSI
                best_match = None
                min_dist_diff = 999.0

                for ws in worker_states:
                    if ws.worker_id in assigned_worker_ids:
                        continue
                    # Compare visual distance to RSSI estimated distance
                    vis_dist = math.sqrt(entity.x**2 + entity.y**2)
                    diff = abs(vis_dist - ws.estimated_dist_m)
                    if diff < min_dist_diff and diff < 6.0:
                        min_dist_diff = diff
                        best_match = ws

                if best_match:
                    entity.entity_id = best_match.worker_id
                    entity.associated_worker_id = best_match.worker_id
                    entity.worker_state = best_match
                    assigned_worker_ids.add(best_match.worker_id)

        # 3. Add Unseen ESP32 Workers (e.g. obscured from camera view or out of FOV)
        for ws in worker_states:
            if ws.worker_id not in assigned_worker_ids:
                # Place in work zone according to estimated RSSI distance and previous coordinates
                fused_entities.append(FusedEntity(
                    entity_id=ws.worker_id,
                    entity_type="worker",
                    x=ws.x,
                    y=ws.y,
                    vx=ws.vx,
                    vy=ws.vy,
                    speed_mps=math.sqrt(ws.vx**2 + ws.vy**2),
                    speed_kmh=round(math.sqrt(ws.vx**2 + ws.vy**2) * 3.6, 1),
                    heading_deg=0.0,
                    predicted_trajectory=[(round(ws.x + ws.vx * t, 2), round(ws.y + ws.vy * t, 2)) for t in [1.0, 2.0, 3.0]],
                    confidence=0.7,
                    associated_worker_id=ws.worker_id,
                    worker_state=ws,
                    lidar_verified=False,
                    last_update=now
                ))

        # 4. Clean up stale EKF filters
        active_ids = {e.entity_id for e in fused_entities}
        self.filters = {k: v for k, v in self.filters.items() if k in active_ids}

        return fused_entities
