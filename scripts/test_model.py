#!/usr/bin/env python3
"""
Model Accuracy & Verification Benchmarking Tool.
Tests YOLO object detection, PPE classification, and FPS latency on camera feeds or test images.
Usage:
  python scripts/test_model.py
  python scripts/test_model.py --source picam2
  python scripts/test_model.py --source 0
  python scripts/test_model.py --model yolov8s.pt --source synthetic --save
"""

import sys
import os
import time
import argparse
import cv2
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import CONFIG
from src.cv.detector import ObjectDetector
from src.sensors.camera_manager import CameraFeed


def test_model(source: str, model_path: str, save_output: bool = True, iterations: int = 20):
    print("=" * 65)
    print(" SMART WORK-ZONE SAFETY POLE - MODEL ACCURACY BENCHMARK ")
    print("=" * 65)

    print(f"[1/4] Initializing Detector with model: {model_path}...")
    detector = ObjectDetector(model_path=model_path)
    print(f"      Active Model: {detector.active_model_name}")
    print(f"      Confidence Thresholds: Worker={getattr(CONFIG.cv, 'WORKER_CONF_THRESHOLD', 0.28)}, Vehicle={getattr(CONFIG.cv, 'VEHICLE_CONF_THRESHOLD', 0.45)}")
    print(f"      Target Classes: Workers={CONFIG.cv.WORKER_CLASSES}, Vehicles={CONFIG.cv.VEHICLE_CLASSES}")

    print(f"\n[2/4] Ingesting test frame from source: '{source}'...")
    frame = None
    if source.lower() in ["synthetic", "sim", "mock"]:
        cf = CameraFeed("TestCam", "synthetic")
        frame = cf._render_synthetic_scene()
    elif source.isdigit():
        cap = cv2.VideoCapture(int(source))
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
    elif source.lower() == "picam2":
        try:
            from picamera2 import Picamera2
            picam = Picamera2()
            cfg = picam.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
            picam.configure(cfg)
            picam.start()
            frame = picam.capture_array()
            picam.stop()
            picam.close()
        except Exception as e:
            print(f"      Picamera2 capture error: {e}. Falling back to synthetic.")
            cf = CameraFeed("TestCam", "synthetic")
            frame = cf._render_synthetic_scene()
    elif os.path.exists(source):
        frame = cv2.imread(source)

    if frame is None:
        print("      Failed to capture frame from source. Using synthetic scene.")
        cf = CameraFeed("TestCam", "synthetic")
        frame = cf._render_synthetic_scene()

    # Frame specs
    print(f"      Frame Shape: {frame.shape}, Channels: {frame.ndim}, Dtype: {frame.dtype}")

    # 3. Benchmark Inference Speed
    print(f"\n[3/4] Running {iterations} inference passes for latency & FPS benchmarking...")
    latencies = []
    detections = []
    is_synth = source.lower() in ["synthetic", "sim", "mock"]

    # Warmup
    _ = detector.detect(frame, is_synthetic=is_synth)

    for i in range(iterations):
        t0 = time.time()
        dets = detector.detect(frame, is_synthetic=is_synth)
        dt = (time.time() - t0) * 1000
        latencies.append(dt)
        if i == 0:
            detections = dets

    avg_ms = np.mean(latencies)
    p95_ms = np.percentile(latencies, 95)
    fps = 1000.0 / max(1.0, avg_ms)

    print(f"      Average Latency: {avg_ms:.2f} ms ({fps:.1f} FPS)")
    print(f"      95th Percentile Latency: {p95_ms:.2f} ms")

    # 4. Display Detections & Classification Results
    print(f"\n[4/4] Detection & Classification Results (Total: {len(detections)}):")
    print("-" * 65)
    print(f" {'CLASS':<10} | {'SUBCLASS':<15} | {'CONF':<8} | {'PPE VERIFIED':<14} | {'BBOX'}")
    print("-" * 65)

    if not detections:
        print(" No objects detected. Consider adjusting confidence threshold or camera angle.")
    else:
        for d in detections:
            ppe_str = f"YES ({int(d.ppe_confidence*100)}%)" if d.ppe_verified else "NO"
            print(f" {d.class_name.upper():<10} | {d.subclass:<15} | {d.confidence*100:>5.1f}% | {ppe_str:<14} | {d.bbox}")

    # Draw and save annotated frame
    if save_output:
        annotated = frame.copy()
        if annotated.ndim == 3 and annotated.shape[2] == 4:
            annotated = cv2.cvtColor(annotated, cv2.COLOR_BGRA2BGR)

        for d in detections:
            x1, y1, x2, y2 = d.bbox
            color = (0, 140, 255) if d.class_name == "vehicle" else (0, 255, 0)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            lbl = f"{d.subclass.upper()} ({int(d.confidence*100)}%)"
            if d.class_name == "worker" and d.ppe_verified:
                lbl += " [PPE]"
            cv2.rectangle(annotated, (x1, max(0, y1 - 20)), (x1 + len(lbl)*9, y1), color, -1)
            cv2.putText(annotated, lbl, (x1 + 3, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

        out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output_detection.jpg")
        cv2.imwrite(out_path, annotated)
        print("-" * 65)
        print(f" Annotated benchmark image saved to: {out_path}")
    print("=" * 65)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Work-Zone Safety Model Benchmark")
    parser.add_argument("--source", type=str, default="synthetic",
                        help="Input source: 'synthetic', 'picam2', camera index (e.g. '0'), or file path")
    parser.add_argument("--model", type=str, default=CONFIG.cv.MODEL_PATH,
                        help="Path or name of model (.pt file)")
    parser.add_argument("--iterations", type=int, default=15,
                        help="Number of benchmark iterations")
    parser.add_argument("--no-save", action="store_true",
                        help="Do not save output image")
    args = parser.parse_args()

    test_model(
        source=args.source,
        model_path=args.model,
        save_output=not args.no_save,
        iterations=args.iterations
    )
