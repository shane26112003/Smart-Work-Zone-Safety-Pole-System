"""
Computer Vision Detector & PPE Classifier Unit Tests.
Verifies YOLO model loading, 4-channel sanitization, PPE verification, and tracking.
"""

import numpy as np
import cv2
from src.config import CONFIG
from src.cv.detector import ObjectDetector, Detection
from src.cv.tracker import MultiObjectTracker, TrackedObject


def test_detector_initialization():
    """Verify detector loads YOLO weights successfully."""
    detector = ObjectDetector()
    assert detector._is_yolo_loaded is True
    assert detector.model is not None
    assert detector.active_model_name in ["yolov8s.pt", "yolo11s.pt", "yolo11n.pt", "yolov8n.pt"]


def test_four_channel_and_grayscale_sanitization():
    """Verify detector handles 4-channel BGRA and 1-channel grayscale images without crashing."""
    detector = ObjectDetector()

    # 1. Test 4-channel BGRA (typical Pi 5 libcamera buffer)
    bgra_frame = np.zeros((480, 640, 4), dtype=np.uint8)
    bgra_frame[:, :, 0] = 50
    bgra_frame[:, :, 1] = 60
    bgra_frame[:, :, 2] = 70
    bgra_frame[:, :, 3] = 255
    dets_4ch = detector.detect(bgra_frame)
    assert isinstance(dets_4ch, list)

    # 2. Test 1-channel grayscale
    gray_frame = np.zeros((480, 640), dtype=np.uint8)
    dets_gray = detector.detect(gray_frame)
    assert isinstance(dets_gray, list)


def test_clahe_contrast_enhancement():
    """Verify CLAHE contrast enhancement increases luminance standard deviation on low-contrast frames."""
    low_contrast = np.full((100, 100, 3), 80, dtype=np.uint8)
    low_contrast[20:40, 20:40] = 95
    enhanced = ObjectDetector.enhance_contrast(low_contrast)
    assert enhanced.shape == (100, 100, 3)
    assert enhanced.dtype == np.uint8


def test_worker_ppe_verification():
    """Verify worker PPE verification detects high-visibility orange and lime vests."""
    # 1. High-vis Orange vest crop
    orange_crop = np.zeros((120, 60, 3), dtype=np.uint8)
    orange_crop[30:70, :] = (0, 140, 255) # Fluorescent Orange BGR
    has_ppe, ppe_score, subcat = ObjectDetector.verify_worker_ppe(orange_crop)
    assert has_ppe is True
    assert ppe_score > 0.0
    assert subcat == "worker_ppe"

    # 2. High-vis Lime / Yellow-Green vest crop
    lime_crop = np.zeros((120, 60, 3), dtype=np.uint8)
    lime_crop[30:70, :] = (50, 230, 200) # Fluorescent Lime BGR
    has_ppe_lime, score_lime, subcat_lime = ObjectDetector.verify_worker_ppe(lime_crop)
    assert has_ppe_lime is True
    assert subcat_lime == "worker_ppe"

    # 3. Civilian clothing (dark blue shirt, no vest)
    civilian_crop = np.zeros((120, 60, 3), dtype=np.uint8)
    civilian_crop[30:70, :] = (90, 40, 20) # Dark navy blue BGR
    has_ppe_civ, score_civ, subcat_civ = ObjectDetector.verify_worker_ppe(civilian_crop)
    assert has_ppe_civ is False
    assert subcat_civ == "worker_no_ppe"


def test_synthetic_scene_worker_vehicle_separation():
    """Verify synthetic detector distinguishes the worker and vehicle from safety cones."""
    from src.sensors.camera_manager import CameraFeed
    cf = CameraFeed("Road_Test", "synthetic")
    frame = cf._render_synthetic_scene()

    detector = ObjectDetector()
    detections = detector.detect(frame, is_synthetic=True)

    class_names = [d.class_name for d in detections]
    assert "worker" in class_names
    assert "vehicle" in class_names

    # Ensure safety cones were NOT counted as 6 additional workers!
    worker_dets = [d for d in detections if d.class_name == "worker"]
    assert len(worker_dets) == 1
    assert worker_dets[0].ppe_verified is True


def test_tracker_subclass_and_ppe_propagation():
    """Verify MultiObjectTracker maintains subclass and PPE metadata across frames."""
    tracker = MultiObjectTracker(min_hits=1)

    dets = [
        Detection(
            bbox=(100, 100, 150, 200),
            confidence=0.91,
            class_id=0,
            class_name="worker",
            subclass="worker_ppe",
            ppe_verified=True,
            ppe_confidence=0.88,
            center=(125, 150),
            bottom_center=(125, 200)
        ),
        Detection(
            bbox=(300, 200, 400, 300),
            confidence=0.89,
            class_id=7,
            class_name="vehicle",
            subclass="truck",
            ppe_verified=False,
            ppe_confidence=0.0,
            center=(350, 250),
            bottom_center=(350, 300)
        )
    ]

    tracks = tracker.update(dets)
    assert len(tracks) == 2

    worker_track = next(t for t in tracks if t.class_name == "worker")
    assert worker_track.subclass == "worker_ppe"
    assert worker_track.ppe_verified is True

    veh_track = next(t for t in tracks if t.class_name == "vehicle")
    assert veh_track.subclass == "truck"
