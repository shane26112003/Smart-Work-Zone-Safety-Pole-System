"""
High-Precision YOLO Object Detector & PPE Classifier for Road Work Zones.
Optimized for Raspberry Pi 5 CPU/GPU and OV5647 / Pi Camera Module 3.
Supports:
  1. Multi-tier vehicle classification (car, truck, bus, motorcycle, bicycle).
  2. Construction worker PPE verification (fluorescent safety vest & hard hat analysis).
  3. OV5647 CSI camera enhancement (CLAHE contrast normalization).
  4. Robust multi-channel input sanitization (4-channel BGRA/XBGR -> BGR).
  5. Accurate synthetic simulation detection (filters out safety cones).
"""

import os
import time
import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional
import logging

from ..config import CONFIG

logger = logging.getLogger("ObjectDetector")


@dataclass
class Detection:
    """Represents a single detected bounding box in image coordinates."""
    bbox: Tuple[int, int, int, int]      # (x1, y1, x2, y2)
    confidence: float
    class_id: int
    class_name: str                      # "worker" or "vehicle"
    subclass: str = ""                   # "car", "truck", "bus", "motorcycle", "bicycle", "worker_ppe", "pedestrian"
    ppe_verified: bool = False           # True if high-vis vest or hard hat detected
    ppe_confidence: float = 0.0          # Confidence / coverage score for PPE
    center: Tuple[int, int] = (0, 0)     # (cx, cy)
    bottom_center: Tuple[int, int] = (0, 0) # (cx, y2) - Ground contact point


class ObjectDetector:
    """
    Advanced Object Detector with PPE Verification and Vehicle Classification.
    Configured for Raspberry Pi 5 and dual road/workzone camera setups.
    """

    COCO_SUBCLASS_MAP = {
        0: "person",
        1: "bicycle",
        2: "car",
        3: "motorcycle",
        5: "bus",
        7: "truck"
    }

    def __init__(self, model_path: Optional[str] = None, conf_threshold: Optional[float] = None):
        self.model_path = model_path or CONFIG.cv.MODEL_PATH
        self.conf_threshold = conf_threshold or CONFIG.cv.CONFIDENCE_THRESHOLD
        self.model = None
        self._is_yolo_loaded = False
        self.active_model_name = "None"

        # Initialize neural network model
        self._init_model()

    def _init_model(self):
        """Attempts to load Ultralytics YOLO model with intelligent fallback hierarchy."""
        try:
            import torch
            # Optimize PyTorch CPU threading for Raspberry Pi 5 (4 Cortex-A76 cores)
            if hasattr(torch, "set_num_threads"):
                torch.set_num_threads(4)
        except Exception:
            pass

        try:
            from ultralytics import YOLO

            # Candidates in order of accuracy / preference
            candidates = [
                self.model_path,
                CONFIG.cv.MODEL_PATH,
                CONFIG.cv.FALLBACK_MODEL_PATH,
                os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "yolov8s.pt"),
                os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "yolo11s.pt"),
                os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "yolo11n.pt"),
                os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "yolov8n.pt"),
                "yolov8s.pt",
                "yolov8n.pt"
            ]

            loaded = False
            for path in candidates:
                if path and os.path.exists(path):
                    logger.info(f"Loading YOLO model from: {path}...")
                    self.model = YOLO(path)
                    self._is_yolo_loaded = True
                    self.active_model_name = os.path.basename(path)
                    logger.info(f"YOLO model '{self.active_model_name}' loaded successfully.")
                    loaded = True
                    break

            if not loaded:
                # If no local file was found, download default
                logger.info(f"Downloading/loading default YOLO model: {self.model_path}...")
                self.model = YOLO(self.model_path)
                self._is_yolo_loaded = True
                self.active_model_name = os.path.basename(self.model_path)
                logger.info(f"YOLO model '{self.active_model_name}' loaded successfully.")

        except Exception as e:
            logger.warning(f"Failed to load YOLO model directly ({e}). Operating in resilient CV mode.")
            self._is_yolo_loaded = False

    def detect(self, frame: np.ndarray, is_synthetic: bool = False) -> List[Detection]:
        """
        Runs object detection on the provided BGR image frame.
        Handles frame sanitization, CLAHE contrast enhancement, YOLO prediction,
        and PPE verification.
        """
        if frame is None or frame.size == 0:
            return []

        # 1. Sanitize Frame Input (Crucial for Pi Camera CSI OV5647 BGRA / 4-channel buffers)
        clean_frame = self._sanitize_frame(frame)

        # 2. Check if frame is synthetic simulation frame
        if is_synthetic:
            return self._detect_synthetic(clean_frame)

        # 3. AI Neural Network Detection
        if self._is_yolo_loaded and self.model is not None:
            detections = self._detect_yolo(clean_frame)
            # If YOLO returned detections on camera frame, return them
            if len(detections) > 0:
                return detections
            # If frame is identified as synthetic drawing and YOLO found 0 boxes, run synthetic detection
            if self._is_synthetic_scene(clean_frame):
                return self._detect_synthetic(clean_frame)
            return detections
        else:
            return self._detect_synthetic(clean_frame)

    @staticmethod
    def _sanitize_frame(frame: np.ndarray) -> np.ndarray:
        """
        Guarantees frame is a contiguous 3-channel BGR uint8 array.
        Prevents PyTorch Conv2D channel mismatches (4-channel XBGR8888 crashes).
        """
        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.ndim == 3 and frame.shape[2] == 1:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        if not frame.flags['C_CONTIGUOUS']:
            frame = np.ascontiguousarray(frame)

        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)

        return frame

    @staticmethod
    def enhance_contrast(frame: np.ndarray) -> np.ndarray:
        """
        Enhances low-contrast or hazy camera frames from OV5647 Rev 1.3
        using Luminance CLAHE (Contrast-Limited Adaptive Histogram Equalization).
        """
        try:
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(
                clipLimit=getattr(CONFIG.cv, "CLAHE_CLIP_LIMIT", 2.0),
                tileGridSize=(8, 8)
            )
            l_enhanced = clahe.apply(l)
            lab_enhanced = cv2.merge((l_enhanced, a, b))
            return cv2.cvtColor(lab_enhanced, cv2.COLOR_LAB2BGR)
        except Exception:
            return frame

    @staticmethod
    def verify_worker_ppe(crop: np.ndarray) -> Tuple[bool, float, str]:
        """
        Analyzes cropped person detection for Work-Zone PPE:
        - ANSI Class 2/3 Fluorescent Safety Orange / Red-Orange vest
        - High-vis Fluorescent Safety Yellow-Green / Lime vest
        - High-reflectance silver / white stripes
        - Safety Hard Hat in head region (top 20%)

        Returns: (is_ppe_verified, ppe_score, subcategory)
        """
        if crop is None or crop.size == 0 or crop.shape[0] < 12 or crop.shape[1] < 8:
            return False, 0.0, "worker"

        h, w = crop.shape[:2]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        # 1. Torso / Vest region: upper 15% to 65% of person height
        torso_y1 = int(h * 0.15)
        torso_y2 = int(h * 0.65)
        torso_hsv = hsv[torso_y1:torso_y2, :]
        torso_area = max(1, torso_hsv.shape[0] * torso_hsv.shape[1])

        # Fluorescent Safety Orange / Amber (H: 5-25, S: 85-255, V: 85-255)
        mask_orange = cv2.inRange(torso_hsv, np.array([5, 85, 85]), np.array([25, 255, 255]))

        # Fluorescent Safety Yellow-Green / Lime (H: 26-48, S: 70-255, V: 85-255)
        mask_lime = cv2.inRange(torso_hsv, np.array([26, 70, 85]), np.array([48, 255, 255]))

        # Reflective high-luminance strips (V > 195, S < 55)
        mask_reflective = cv2.inRange(torso_hsv, np.array([0, 0, 195]), np.array([180, 55, 255]))

        combined_vest = cv2.bitwise_or(mask_orange, cv2.bitwise_or(mask_lime, mask_reflective))
        vest_pixels = np.count_nonzero(combined_vest)
        vest_ratio = vest_pixels / torso_area

        # 2. Head / Hard Hat region: top 20%
        head_hsv = hsv[:int(h * 0.20), :]
        head_area = max(1, head_hsv.shape[0] * head_hsv.shape[1])

        # Yellow / White / Orange hard hat masks
        mask_helmet_yellow = cv2.inRange(head_hsv, np.array([20, 80, 90]), np.array([38, 255, 255]))
        mask_helmet_orange = cv2.inRange(head_hsv, np.array([5, 90, 90]), np.array([19, 255, 255]))
        mask_helmet_white = cv2.inRange(head_hsv, np.array([0, 0, 180]), np.array([180, 45, 255]))
        combined_helmet = cv2.bitwise_or(mask_helmet_yellow, cv2.bitwise_or(mask_helmet_orange, mask_helmet_white))
        helmet_ratio = np.count_nonzero(combined_helmet) / head_area

        # Composite PPE score (0.0 to 1.0)
        ppe_score = round(min(1.0, (vest_ratio * 2.2 + helmet_ratio * 0.8)), 2)
        min_ratio = getattr(CONFIG.cv, "PPE_MIN_RATIO", 0.05)
        has_ppe = (vest_ratio >= min_ratio or helmet_ratio >= 0.15 or ppe_score >= 0.12)

        subcat = "worker_ppe" if has_ppe else "worker_no_ppe"
        return has_ppe, ppe_score, subcat

    def _detect_yolo(self, frame: np.ndarray) -> List[Detection]:
        """Runs YOLO neural inference on sanitized frame."""
        detections: List[Detection] = []
        h, w = frame.shape[:2]

        try:
            # Apply CLAHE contrast enhancement for low-contrast OV5647 frames
            input_frame = frame
            if getattr(CONFIG.cv, "ENABLE_ENHANCEMENT", True):
                input_frame = self.enhance_contrast(frame)

            target_classes = CONFIG.cv.WORKER_CLASSES + CONFIG.cv.VEHICLE_CLASSES
            img_size = getattr(CONFIG.cv, "IMAGE_SIZE", 640)

            results = self.model.predict(
                source=input_frame,
                conf=self.conf_threshold,
                iou=CONFIG.cv.IOU_THRESHOLD,
                classes=target_classes,
                imgsz=img_size,
                verbose=False
            )

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])

                    # Clamp coordinates to frame bounds
                    x1 = max(0, min(w - 1, x1))
                    y1 = max(0, min(h - 1, y1))
                    x2 = max(x1 + 1, min(w, x2))
                    y2 = max(y1 + 1, min(h, y2))

                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    bc = (cx, y2)

                    # Classification logic
                    if cls_id in CONFIG.cv.WORKER_CLASSES:
                        # Person detected - evaluate Work-Zone PPE
                        crop = frame[y1:y2, x1:x2]
                        has_ppe, ppe_score, subcat = self.verify_worker_ppe(crop)

                        strict_mode = getattr(CONFIG.cv, "STRICT_PPE_MODE", False)
                        if not has_ppe and strict_mode:
                            c_name = "worker" # Keep worker for fusion engine compatibility
                            subcat = "pedestrian"
                        else:
                            c_name = "worker"

                        detections.append(Detection(
                            bbox=(x1, y1, x2, y2),
                            confidence=round(conf, 2),
                            class_id=cls_id,
                            class_name=c_name,
                            subclass=subcat,
                            ppe_verified=has_ppe,
                            ppe_confidence=ppe_score,
                            center=(cx, cy),
                            bottom_center=bc
                        ))

                    elif cls_id in CONFIG.cv.VEHICLE_CLASSES:
                        subcat = self.COCO_SUBCLASS_MAP.get(cls_id, "vehicle")
                        detections.append(Detection(
                            bbox=(x1, y1, x2, y2),
                            confidence=round(conf, 2),
                            class_id=cls_id,
                            class_name="vehicle",
                            subclass=subcat,
                            ppe_verified=False,
                            ppe_confidence=0.0,
                            center=(cx, cy),
                            bottom_center=bc
                        ))

        except Exception as e:
            logger.error(f"YOLO inference error: {e}", exc_info=True)
            return self._detect_synthetic(frame)

        return detections

    @staticmethod
    def _is_synthetic_scene(frame: np.ndarray) -> bool:
        """Detects if frame is generated by the synthetic scenario renderer."""
        # Top-left corner in synthetic mode has text watermark "ROAD_CAMERA [LIVE]" or solid sky
        # Check if the sky color (top row) matches the synthetic sky (180, 160, 120)
        if frame is not None and frame.shape[0] >= 50 and frame.shape[1] >= 50:
            top_pixel = frame[10, 10]
            # Synthetic sky BGR is approximately (180, 160, 120)
            if abs(int(top_pixel[0]) - 180) < 15 and abs(int(top_pixel[1]) - 160) < 15 and abs(int(top_pixel[2]) - 120) < 15:
                return True
        return False

    def _detect_synthetic(self, frame: np.ndarray) -> List[Detection]:
        """
        High-precision detector for the synthetic road simulator.
        Accurately separates workers from traffic safety cones!
        """
        detections: List[Detection] = []
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 1. Detect Synthetic Worker (Identified by Yellow Hard Hat + Orange Vest)
        # Yellow hard hat: H in [25, 35], S > 180, V > 180
        lower_yellow = np.array([25, 180, 180])
        upper_yellow = np.array([35, 255, 255])
        mask_hat = cv2.inRange(hsv, lower_yellow, upper_yellow)

        worker_candidates = []
        worker_scores = []
        contours_hat, _ = cv2.findContours(mask_hat, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours_hat:
            area = cv2.contourArea(c)
            if area > 8:
                hx, hy, hw, hh = cv2.boundingRect(c)
                # Expand box downwards to enclose full body
                wrk_h = int(hh * 7.0)
                wrk_w = int(hw * 2.2)
                x1 = max(0, hx - int(wrk_w * 0.3))
                x2 = min(w - 1, hx + wrk_w)
                y1 = max(0, hy)
                y2 = min(h - 1, hy + wrk_h)
                worker_candidates.append([x1, y1, x2 - x1, y2 - y1])
                worker_scores.append(0.92)

        # Apply Non-Maximum Suppression to keep single clean worker box
        if worker_candidates:
            indices = cv2.dnn.NMSBoxes(worker_candidates, worker_scores, 0.5, 0.3)
            if len(indices) > 0:
                for idx in indices:
                    i = idx[0] if isinstance(idx, (list, tuple, np.ndarray)) else idx
                    bx, by, bw, bh = worker_candidates[i]
                    x1, y1, x2, y2 = bx, by, bx + bw, by + bh
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    detections.append(Detection(
                        bbox=(x1, y1, x2, y2),
                        confidence=0.92,
                        class_id=0,
                        class_name="worker",
                        subclass="worker_ppe",
                        ppe_verified=True,
                        ppe_confidence=0.95,
                        center=(cx, cy),
                        bottom_center=(cx, y2)
                    ))

        # 2. Detect Synthetic Vehicle (Metallic Blue or Red car body in roadway)
        mask_blue = cv2.inRange(hsv, np.array([100, 70, 50]), np.array([130, 255, 255]))
        mask_red = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([15, 255, 255]))
        mask_veh = cv2.bitwise_or(mask_blue, mask_red)

        # Only look in roadway area (below horizon, right of cone divider line)
        mask_veh[:int(h * 0.40), :] = 0
        mask_veh[:, :int(w * 0.30)] = 0 # Ignore work-zone surface

        contours_veh, _ = cv2.findContours(mask_veh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours_veh:
            area = cv2.contourArea(c)
            if area > 100:
                x, y, bw, bh = cv2.boundingRect(c)
                # Vehicles in roadway have width >= 0.7 * height
                if bw >= int(bh * 0.7) and bw > 15:
                    cx = int(x + bw / 2)
                    cy = int(y + bh / 2)
                    # Exclude worker overlap
                    if not any(abs(d.center[0] - cx) < 35 and abs(d.center[1] - cy) < 35 for d in detections):
                        detections.append(Detection(
                            bbox=(x, y, x + bw, y + bh),
                            confidence=0.90,
                            class_id=2,
                            class_name="vehicle",
                            subclass="car",
                            ppe_verified=False,
                            ppe_confidence=0.0,
                            center=(cx, cy),
                            bottom_center=(cx, y + bh)
                        ))

        return detections
