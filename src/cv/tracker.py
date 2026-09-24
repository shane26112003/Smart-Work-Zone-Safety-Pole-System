"""
Multi-Object Tracker (Kalman Filter + IoU Data Association).
Maintains persistent identities, velocity estimation, and trajectory histories
for detected workers and vehicles across successive video frames.
"""

import time
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional, Deque
from scipy.optimize import linear_sum_assignment

from .detector import Detection
from ..config import CONFIG

@dataclass
class TrackedObject:
    """Represents an active track across time."""
    track_id: int
    class_name: str                 # "worker" or "vehicle"
    bbox: Tuple[int, int, int, int] # (x1, y1, x2, y2)
    centroid: Tuple[int, int]       # (cx, cy)
    ground_point: Tuple[int, int]   # (cx, y2)
    velocity_px_s: Tuple[float, float] = (0.0, 0.0) # (vx, vy)
    history: Deque[Tuple[int, int]] = field(default_factory=lambda: deque(maxlen=30))
    age: int = 1
    hits: int = 1
    time_since_update: int = 0
    confidence: float = 0.8
    subclass: str = ""              # "car", "truck", "bus", "motorcycle", "bicycle", "worker_ppe", "pedestrian"
    ppe_verified: bool = False      # Verified high-visibility vest or hard hat
    ppe_confidence: float = 0.0
    associated_worker_id: Optional[str] = None # Filled if matched to ESP32


class KalmanBoxTracker:
    """
    Kalman Filter for tracking bounding boxes in image space [u, v, s, r]
    where u, v is center, s is scale/area, r is aspect ratio.
    """
    count = 0

    def __init__(self, bbox: Tuple[int, int, int, int], class_name: str, confidence: float,
                 subclass: str = "", ppe_verified: bool = False, ppe_confidence: float = 0.0):
        # State vector: [x, y, s, r, vx, vy, vs]
        self.kf_dim_x = 7
        self.kf_dim_z = 4

        self.x = np.zeros((7, 1))
        # Initial measurement
        w = max(1, bbox[2] - bbox[0])
        h = max(1, bbox[3] - bbox[1])
        self.x[0] = bbox[0] + w / 2.0
        self.x[1] = bbox[1] + h / 2.0
        self.x[2] = w * h
        self.x[3] = w / float(h)

        # State Transition Matrix F
        self.F = np.eye(7)
        self.F[0, 4] = 1.0 # x + vx*dt
        self.F[1, 5] = 1.0 # y + vy*dt
        self.F[2, 6] = 1.0 # s + vs*dt

        # Measurement Matrix H
        self.H = np.zeros((4, 7))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0
        self.H[3, 3] = 1.0

        # Covariance Matrices P, Q, R
        self.P = np.diag([10.0, 10.0, 100.0, 10.0, 100.0, 100.0, 1000.0])
        self.Q = np.diag([1.0, 1.0, 1.0, 1.0, 0.01, 0.01, 0.0001])
        self.R = np.diag([1.0, 1.0, 10.0, 10.0])

        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1

        self.class_name = class_name
        self.subclass = subclass
        self.ppe_verified = ppe_verified
        self.ppe_confidence = ppe_confidence
        self.confidence = confidence
        self.history: Deque[Tuple[int, int]] = deque(maxlen=30)
        self.hits = 1
        self.age = 1
        self.time_since_update = 0
        self.associated_worker_id = None
        self.last_update_time = time.time()

        cx = int(self.x[0, 0])
        cy = int(self.x[1, 0])
        self.history.append((cx, cy))

    def predict(self) -> Tuple[int, int, int, int]:
        """Predicts the bounding box for current time step."""
        if (self.x[6] + self.x[2]) <= 0:
            self.x[6] = 0.0

        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        self.age += 1
        if self.time_since_update > 0:
            self.hits = 0
        self.time_since_update += 1

        return self.get_state()

    def update(self, bbox: Tuple[int, int, int, int], confidence: float,
               subclass: str = "", ppe_verified: bool = False, ppe_confidence: float = 0.0):
        """Updates the filter with an observed bounding box."""
        now = time.time()
        self.time_since_update = 0
        self.hits += 1
        self.confidence = confidence
        if subclass:
            self.subclass = subclass
        self.ppe_verified = ppe_verified
        self.ppe_confidence = ppe_confidence

        w = max(1, bbox[2] - bbox[0])
        h = max(1, bbox[3] - bbox[1])
        z = np.array([[bbox[0] + w / 2.0], [bbox[1] + h / 2.0], [w * h], [w / float(h)]])

        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

        cx = int(self.x[0, 0])
        cy = int(self.x[1, 0])
        self.history.append((cx, cy))
        self.last_update_time = now

    def get_state(self) -> Tuple[int, int, int, int]:
        """Converts [x, y, s, r] back to (x1, y1, x2, y2)."""
        w = np.sqrt(max(1.0, self.x[2, 0] * self.x[3, 0]))
        h = max(1.0, self.x[2, 0] / w)
        x1 = int(self.x[0, 0] - w / 2.0)
        y1 = int(self.x[1, 0] - h / 2.0)
        x2 = int(self.x[0, 0] + w / 2.0)
        y2 = int(self.x[1, 0] + h / 2.0)
        return (x1, y1, x2, y2)


def compute_iou(box1: Tuple[int, int, int, int], box2: Tuple[int, int, int, int]) -> float:
    """Computes Intersection over Union between two boxes."""
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union = area1 + area2 - inter

    return inter / union if union > 0 else 0.0


class MultiObjectTracker:
    """Multi-object tracker associating YOLO detections across frames."""

    def __init__(self, max_age: int = 30, min_hits: int = 3, iou_threshold: float = 0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.trackers: List[KalmanBoxTracker] = []

    def update(self, detections: List[Detection]) -> List[TrackedObject]:
        """
        Updates trackers with current frame detections.
        Returns list of active TrackedObject instances.
        """
        # 1. Predict new positions for existing trackers
        predicted_boxes = []
        for trk in self.trackers:
            predicted_boxes.append(trk.predict())

        # 2. Compute IoU Cost Matrix
        n_tracks = len(self.trackers)
        n_dets = len(detections)
        cost_matrix = np.zeros((n_tracks, n_dets))

        for t_idx, trk in enumerate(self.trackers):
            for d_idx, det in enumerate(detections):
                # Only match if classes align
                if trk.class_name == det.class_name:
                    iou = compute_iou(predicted_boxes[t_idx], det.bbox)
                    cost_matrix[t_idx, d_idx] = 1.0 - iou
                else:
                    cost_matrix[t_idx, d_idx] = 1.0 # Maximum cost

        matched_tracks, matched_dets = [], []
        if n_tracks > 0 and n_dets > 0:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r, c] < (1.0 - self.iou_threshold):
                    matched_tracks.append(r)
                    matched_dets.append(c)
                    self.trackers[r].update(
                        detections[c].bbox,
                        detections[c].confidence,
                        subclass=detections[c].subclass,
                        ppe_verified=detections[c].ppe_verified,
                        ppe_confidence=detections[c].ppe_confidence
                    )

        # 3. Create new trackers for unmatched detections
        for d_idx, det in enumerate(detections):
            if d_idx not in matched_dets:
                new_trk = KalmanBoxTracker(
                    det.bbox,
                    det.class_name,
                    det.confidence,
                    subclass=det.subclass,
                    ppe_verified=det.ppe_verified,
                    ppe_confidence=det.ppe_confidence
                )
                self.trackers.append(new_trk)

        # 4. Filter expired trackers and build output list
        active_tracks: List[TrackedObject] = []
        surviving_trackers = []

        for trk in self.trackers:
            if trk.time_since_update <= self.max_age:
                surviving_trackers.append(trk)
                if trk.hits >= self.min_hits or trk.age <= 3:
                    state_box = trk.get_state()
                    w = max(1, state_box[2] - state_box[0])
                    h = max(1, state_box[3] - state_box[1])
                    cx = int(state_box[0] + w / 2)
                    cy = int(state_box[1] + h / 2)
                    bc = (cx, state_box[3]) # Ground touch point

                    # Calculate velocity in pixels/second
                    vx = float(trk.x[4, 0]) * CONFIG.camera.TARGET_FPS
                    vy = float(trk.x[5, 0]) * CONFIG.camera.TARGET_FPS

                    active_tracks.append(TrackedObject(
                        track_id=trk.id,
                        class_name=trk.class_name,
                        bbox=state_box,
                        centroid=(cx, cy),
                        ground_point=bc,
                        velocity_px_s=(vx, vy),
                        history=trk.history,
                        age=trk.age,
                        hits=trk.hits,
                        time_since_update=trk.time_since_update,
                        confidence=trk.confidence,
                        subclass=trk.subclass,
                        ppe_verified=trk.ppe_verified,
                        ppe_confidence=trk.ppe_confidence,
                        associated_worker_id=trk.associated_worker_id
                    ))

        self.trackers = surviving_trackers
        return active_tracks
