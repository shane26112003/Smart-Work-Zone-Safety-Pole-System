"""
Bird's-Eye View (BEV) Homography & Coordinate Transformation Module.
Maps camera image pixels (u, v) to metric ground-plane coordinates (X, Y) in meters,
referenced to the Safety Pole at (0, 0).
"""

import cv2
import numpy as np
from typing import Tuple, List, Optional
import logging

from ..config import CONFIG

logger = logging.getLogger("BEVTransform")


class BEVTransform:
    """
    Transforms between Camera Perspective (pixels) and Bird's-Eye View (meters).
    X-axis (meters): Lateral across road & work zone (Negative = Work Zone, Positive = Road Lane)
    Y-axis (meters): Longitudinal along road (0 = at pole, Positive = Approaching traffic)
    """

    def __init__(self, image_width: int = 640, image_height: int = 480):
        self.width = image_width
        self.height = image_height

        # Define 4 reference calibration points in Camera Pixel Space
        # Selected based on camera mount angle (~2.5m elevation, ~15 deg downward pitch)
        self.src_pts = np.float32([
            [int(self.width * 0.40), int(self.height * 0.45)], # Far-left (cone line far, Y=35m)
            [int(self.width * 0.70), int(self.height * 0.45)], # Far-right (outer lane far, Y=35m)
            [int(self.width * 0.15), int(self.height * 0.95)], # Near-left (cone line near, Y=2m)
            [int(self.width * 0.95), int(self.height * 0.95)]  # Near-right (outer lane near, Y=2m)
        ])

        # Corresponding 4 points in Metric Ground Plane (meters relative to Safety Pole)
        self.dst_pts = np.float32([
            [CONFIG.geometry.HAZARD_LINE_X, 35.0],                           # Far cone divider
            [CONFIG.geometry.ROAD_LANE_X_MAX, 35.0],                         # Far outer lane
            [CONFIG.geometry.HAZARD_LINE_X, 2.0],                            # Near cone divider
            [CONFIG.geometry.ROAD_LANE_X_MAX, 2.0]                           # Near outer lane
        ])

        # Compute Forward & Inverse Homography Matrices
        self.H_img2metric = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)
        self.H_metric2img = cv2.getPerspectiveTransform(self.dst_pts, self.src_pts)

    def image_to_metric(self, px: float, py: float) -> Tuple[float, float]:
        """
        Converts pixel coordinate (u, v) (typically ground contact point cx, y2)
        to metric ground position (X, Y) in meters.
        """
        pt = np.array([[[px, py]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.H_img2metric)
        x_m = float(transformed[0, 0, 0])
        y_m = float(transformed[0, 0, 1])

        # Clamping to reasonable detection envelope
        x_m = max(-15.0, min(15.0, x_m))
        y_m = max(0.0, min(80.0, y_m))
        return (round(x_m, 2), round(y_m, 2))

    def metric_to_image(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """
        Converts metric ground coordinates (X, Y) to camera image pixel (u, v).
        Useful for projecting radar tracks, danger zones, or LiDAR rays onto video frames.
        """
        pt = np.array([[[x_m, y_m]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.H_metric2img)
        u = int(transformed[0, 0, 0])
        v = int(transformed[0, 0, 1])
        return (u, v)
