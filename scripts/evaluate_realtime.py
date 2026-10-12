#!/usr/bin/env python3
"""
Real-Time Hardware Evaluation & Benchmarking Tool with Live Video Output.
Evaluates YOLO object detection, LiDAR-Camera fusion, ESP32-S3 IMU telemetry,
and Collision Risk models using live sensor data streams.

Supports:
  1. Live Interactive Video Display Window (--display)
  2. Annotated Output Video Recording (--record)
  3. Pre-recorded Traffic Video Ingestion (--camera path/to/video.mp4)
  4. Real Hardware Live Streaming (Pi CSI 'picam2', USB '0', ESP32 UDP sensors)

Usage:
  # Watch live video with bounding boxes & hazard overlay on screen:
  python scripts/evaluate_realtime.py --camera picam2 --display

  # Record output video to .mp4 while evaluating:
  python scripts/evaluate_realtime.py --camera picam2 --record --duration 30

  # Display and Record simultaneously on pre-recorded video:
  python scripts/evaluate_realtime.py --camera test_traffic.mp4 --display --record
"""

import sys
import os
import time
import argparse
import datetime
import json
import numpy as np
import cv2

# Add root directory to sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import CONFIG
from src.sensors.lidar_tsd20 import LidarTSD20
from src.sensors.worker_receiver import WorkerReceiver
from src.sensors.camera_manager import CameraManager
from src.cv.detector import ObjectDetector
from src.cv.tracker import MultiObjectTracker
from src.fusion.bev_transform import BEVTransform
from src.fusion.ekf_fusion import SensorFusionEngine
from src.risk.risk_engine import RiskEngine


def run_realtime_evaluation(
    camera_source: str,
    model_path: str,
    duration_sec: float = 30.0,
    save_dir: str = "eval_results",
    save_snapshots: bool = True,
    display: bool = False,
    record: bool = False
):
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(save_dir, exist_ok=True)
    snapshot_dir = os.path.join(save_dir, "snapshots")
    if save_snapshots:
        os.makedirs(snapshot_dir, exist_ok=True)

    print("=" * 70)
    print(" SMART WORK-ZONE SAFETY POLE - REAL-TIME HARDWARE EVALUATION ")
    print("=" * 70)
    print(f" Camera Source:       {camera_source}")
    print(f" YOLO Model:          {model_path}")
    print(f" Evaluation Duration: {'Continuous (Ctrl+C to stop)' if duration_sec <= 0 else f'{duration_sec:.1f} seconds'}")
    print(f" Live Video Window:   {'ENABLED (Press Q/ESC in window to exit)' if display else 'Disabled (Use --display to show)'}")
    print(f" Video Recording:     {'ENABLED (.mp4 output)' if record else 'Disabled (Use --record to save)'}")
    print(f" Sensor Ports:        LiDAR UDP 5006 | Worker UDP 5005")
    print(f" Output Directory:    {save_dir}")
    print("=" * 70)

    # 1. Initialize Hardware Ingestion & AI Perception Layers
    print("[1/5] Initializing Sensors & Vision Pipelines...")
    lidar = LidarTSD20(mode="wifi")
    worker_rx = WorkerReceiver()
    camera_mgr = CameraManager(road_cam_id=camera_source)
    detector = ObjectDetector(model_path=model_path)
    tracker = MultiObjectTracker(min_hits=1)
    bev = BEVTransform(CONFIG.camera.FRAME_WIDTH, CONFIG.camera.FRAME_HEIGHT)
    fusion = SensorFusionEngine(bev)
    risk_engine = RiskEngine()

    # Video Recorder Setup
    video_writer = None
    output_video_path = None
    if record:
        output_video_path = os.path.join(save_dir, f"eval_video_{timestamp_str}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(
            output_video_path, fourcc, 20.0,
            (CONFIG.camera.FRAME_WIDTH, CONFIG.camera.FRAME_HEIGHT)
        )
        print(f"[INFO] Video recording armed -> {output_video_path}")

    # 2. Start Sensor Background Threads
    print("[2/5] Starting live sensor streams...")
    lidar.start()
    worker_rx.start()
    camera_mgr.start()

    time.sleep(1.0)  # Settle streams

    # Metrics Accumulators
    frame_count = 0
    inference_times_ms = []
    loop_times_ms = []
    
    detection_counts = {"worker": 0, "vehicle": 0}
    vehicle_subclasses = {}
    worker_ppe_counts = {"verified": 0, "no_ppe": 0}
    confidences = []

    lidar_packets_seen = 0
    lidar_distances = []
    lidar_speeds = []
    last_lidar_seq = None

    worker_packets_seen = 0
    worker_states = {}
    worker_svms = []
    worker_rssis = []

    risk_level_durations = {"SAFE": 0, "CAUTION": 0, "HIGH RISK": 0, "CRITICAL": 0}
    min_ttc_observed = float("inf")
    active_tracks_counts = []
    fused_associations_count = 0

    print("[3/5] Commencing real-time data ingestion & model evaluation...")
    print("----------------------------------------------------------------------")
    print("  TIME   |  FPS  | INFER(ms) | DETECTIONS | LIDAR (m) | WORKER | RISK")
    print("----------------------------------------------------------------------")

    start_time = time.time()
    last_print_time = start_time
    last_snapshot_time = 0
    saved_snapshot_count = 0

    try:
        while True:
            now = time.time()
            elapsed_total = now - start_time
            if duration_sec > 0 and elapsed_total >= duration_sec:
                break

            loop_start = time.time()

            # A. Ingest live camera frame
            frame = camera_mgr.get_road_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            frame_count += 1

            # B. Ingest live LiDAR
            lidar_reading = lidar.get_latest_reading()
            if lidar_reading and getattr(lidar_reading, "valid", False):
                t_stamp = getattr(lidar_reading, "timestamp", 0)
                if last_lidar_seq != t_stamp:
                    last_lidar_seq = t_stamp
                    lidar_packets_seen += 1
                    lidar_distances.append(lidar_reading.distance_m)
                    lidar_speeds.append(getattr(lidar_reading, "range_rate_mps", 0.0))

            # C. Ingest live Worker Tag
            workers = worker_rx.get_all_active_workers()
            if workers:
                for w_id, w_state in workers.items():
                    worker_packets_seen += 1
                    st_name = getattr(w_state, "motion_state", "UNKNOWN")
                    worker_states[st_name] = worker_states.get(st_name, 0) + 1
                    if hasattr(w_state, "svm"):
                        worker_svms.append(w_state.svm)
                    if hasattr(w_state, "rssi"):
                        worker_rssis.append(w_state.rssi)

            # D. Evaluate YOLO Object Detector
            t_infer_0 = time.time()
            is_synth = camera_mgr.is_road_synthetic()
            detections = detector.detect(frame, is_synthetic=is_synth)
            infer_ms = (time.time() - t_infer_0) * 1000.0
            inference_times_ms.append(infer_ms)

            # Accumulate detection stats
            for d in detections:
                confidences.append(d.confidence)
                if d.class_name == "worker":
                    detection_counts["worker"] += 1
                    if d.ppe_verified:
                        worker_ppe_counts["verified"] += 1
                    else:
                        worker_ppe_counts["no_ppe"] += 1
                elif d.class_name == "vehicle":
                    detection_counts["vehicle"] += 1
                    sub = d.subclass or "car"
                    vehicle_subclasses[sub] = vehicle_subclasses.get(sub, 0) + 1

            # E. Evaluate Tracking & EKF Fusion
            tracks = tracker.update(detections)
            active_tracks_counts.append(len(tracks))

            fused_entities = fusion.fuse(tracks, lidar_reading, workers)
            for fe in fused_entities:
                if getattr(fe, "lidar_verified", False):
                    fused_associations_count += 1

            # F. Evaluate Collision Risk Engine
            assessment = risk_engine.evaluate(fused_entities)
            lvl = assessment.overall_level
            risk_level_durations[lvl] = risk_level_durations.get(lvl, 0) + 1
            if assessment.min_ttc_sec < min_ttc_observed:
                min_ttc_observed = assessment.min_ttc_sec

            loop_ms = (time.time() - loop_start) * 1000.0
            loop_times_ms.append(loop_ms)
            cur_fps = 1000.0 / max(0.1, loop_ms)

            # G. Periodic Console Heartbeat (every 1.0s)
            if now - last_print_time >= 1.0:
                recent_fps = 1000.0 / max(0.1, np.mean(loop_times_ms[-15:]))
                recent_infer = np.mean(inference_times_ms[-15:])
                det_summary = f"{len([d for d in detections if d.class_name=='vehicle'])}V/{len([d for d in detections if d.class_name=='worker'])}W"
                lidar_str = f"{lidar_reading.distance_m:.2f}m" if lidar_reading else "N/A"
                worker_str = list(workers.values())[0].motion_state if workers else "OFFLINE"
                
                print(f" {elapsed_total:>5.1f}s | {recent_fps:>4.1f} | {recent_infer:>6.1f}ms | {det_summary:>10} | {lidar_str:>9} | {worker_str:>6} | {lvl}")
                last_print_time = now

            # H. Video Annotation (for Display, Recording, and Snapshots)
            annotated = None
            if display or record or save_snapshots:
                annotated = frame.copy()
                h, w = annotated.shape[:2]

                # 1. Top Hazard Risk Banner
                banner_colors = {
                    "SAFE": (34, 197, 94),       # Green
                    "CAUTION": (234, 179, 8),    # Yellow
                    "HIGH RISK": (249, 115, 22), # Orange
                    "CRITICAL": (239, 68, 68)    # Red
                }
                b_color = banner_colors.get(lvl, (100, 100, 100))
                cv2.rectangle(annotated, (0, 0), (w, 34), b_color, -1)
                cv2.putText(
                    annotated,
                    f"RISK: {lvl} | TTC: {assessment.min_ttc_sec:.1f}s | DIST: {assessment.min_distance_m:.1f}m",
                    (12, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 0, 0) if lvl == "CAUTION" else (255, 255, 255), 2
                )

                # 2. Draw Object Detections & Tracking Boxes
                for trk in tracks:
                    x1, y1, x2, y2 = trk.bbox
                    is_veh = (trk.class_name == "vehicle")
                    box_color = (0, 140, 255) if is_veh else (0, 255, 0)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, 2)
                    
                    # Tag text
                    if is_veh:
                        sub = getattr(trk, "subclass", "car")
                        tag = f"{sub.upper()} #{trk.track_id} ({int(trk.confidence*100)}%)"
                    else:
                        tag = f"WORKER #{trk.track_id} ({int(trk.confidence*100)}%)"
                        if getattr(trk, "ppe_verified", False):
                            tag += " [PPE VEST]"
                    
                    cv2.rectangle(annotated, (x1, max(0, y1 - 20)), (x1 + len(tag) * 8 + 6, y1), box_color, -1)
                    cv2.putText(annotated, tag, (x1 + 3, max(14, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)

                # 3. Bottom HUD Overlay: Hardware Sensors & Performance
                cv2.rectangle(annotated, (0, h - 28), (w, h), (20, 20, 20), -1)
                lid_text = f"LiDAR: {lidar_reading.distance_m:.2f}m ({lidar_reading.range_rate_mps:+.1f}m/s)" if lidar_reading and getattr(lidar_reading, "valid", False) else "LiDAR: Waiting UDP"
                wrk_text = f"Worker: {list(workers.values())[0].motion_state} (SVM: {list(workers.values())[0].svm:.2f}g)" if workers else "Worker: Offline"
                hud_str = f"{lid_text} | {wrk_text} | FPS: {cur_fps:.1f}"
                cv2.putText(annotated, hud_str, (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

            # I. Write frame to Output Video File
            if video_writer and annotated is not None:
                video_writer.write(annotated)

            # J. Live Interactive Display Window
            if display and annotated is not None:
                cv2.imshow("Safety Pole - Real-Time Model Evaluation", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), ord('Q'), 27):  # 'q' or ESC
                    print("\n[INFO] User pressed 'q' in video display window. Exiting...")
                    break

            # K. Save snapshot periodically if detections or hazard present
            if save_snapshots and annotated is not None and (now - last_snapshot_time >= 5.0 or lvl in ["HIGH RISK", "CRITICAL"]):
                if detections or lvl != "SAFE":
                    last_snapshot_time = now
                    snap_file = os.path.join(snapshot_dir, f"snap_{saved_snapshot_count:04d}_{lvl}.jpg")
                    cv2.imwrite(snap_file, annotated)
                    saved_snapshot_count += 1

            # Frame rate pacing (~25 FPS target)
            time.sleep(max(0.002, 0.040 - (time.time() - loop_start)))

    except KeyboardInterrupt:
        print("\n[INFO] Real-time evaluation interrupted by user.")
    finally:
        # Clean shutdown of display and recording
        if video_writer:
            video_writer.release()
            print(f"[INFO] Output video successfully saved to: {output_video_path}")
        if display:
            cv2.destroyAllWindows()

        # Clean shutdown of sensors
        camera_mgr.stop()
        worker_rx.stop()
        lidar.stop()

    total_duration = max(0.1, time.time() - start_time)

    # 4. Compute Comprehensive Evaluation Metrics
    avg_fps = frame_count / total_duration
    avg_infer_ms = float(np.mean(inference_times_ms)) if inference_times_ms else 0.0
    p95_infer_ms = float(np.percentile(inference_times_ms, 95)) if inference_times_ms else 0.0
    min_infer_ms = float(np.min(inference_times_ms)) if inference_times_ms else 0.0
    max_infer_ms = float(np.max(inference_times_ms)) if inference_times_ms else 0.0

    lidar_rate_hz = lidar_packets_seen / total_duration
    worker_rate_hz = worker_packets_seen / total_duration

    total_workers_detected = detection_counts["worker"]
    ppe_compliance_pct = (worker_ppe_counts["verified"] / max(1, total_workers_detected)) * 100.0

    # 5. Print Evaluation Summary Report to Console
    print("\n" + "=" * 70)
    print(" REAL-TIME MODEL EVALUATION REPORT ")
    print("=" * 70)
    print(f" Total Session Duration:      {total_duration:.2f} seconds")
    print(f" Total Frames Processed:      {frame_count} frames")
    print(f" Effective Framerate:         {avg_fps:.2f} FPS")
    print(f"\n [A] COMPUTER VISION & YOLO LATENCY")
    print(f"     Active Model:            {detector.active_model_name}")
    print(f"     Mean Inference Time:     {avg_infer_ms:.2f} ms")
    print(f"     95th Percentile Latency: {p95_infer_ms:.2f} ms")
    print(f"     Latency Range (Min/Max): {min_infer_ms:.2f} ms / {max_infer_ms:.2f} ms")
    print(f"\n [B] REAL-TIME DETECTION METRICS")
    print(f"     Total Vehicles Detected: {detection_counts['vehicle']}")
    print(f"     Vehicle Subclasses:      {vehicle_subclasses}")
    print(f"     Total Workers Detected:  {detection_counts['worker']}")
    print(f"     PPE Compliance Rate:     {ppe_compliance_pct:.1f}% ({worker_ppe_counts['verified']} verified with high-vis vest)")
    print(f"     Mean Confidence:         {np.mean(confidences)*100:.1f}%" if confidences else "     Mean Confidence:         N/A")
    print(f"\n [C] HARDWARE SENSOR PERFORMANCE")
    print(f"     LiDAR UDP Packets:       {lidar_packets_seen} (Rate: {lidar_rate_hz:.1f} Hz, Target: 50 Hz)")
    if lidar_distances:
        print(f"     LiDAR Measured Range:    {min(lidar_distances):.2f}m to {max(lidar_distances):.2f}m")
    print(f"     Worker Tag Packets:      {worker_packets_seen} (Rate: {worker_rate_hz:.1f} Hz, Target: 25 Hz)")
    print(f"     Worker Motion States:    {worker_states}")
    if worker_svms:
        print(f"     Mean Worker IMU SVM:     {np.mean(worker_svms):.2f} G")
    print(f"\n [D] MULTI-SENSOR FUSION & RISK EVALUATION")
    print(f"     Average Active Tracks:   {np.mean(active_tracks_counts):.2f}" if active_tracks_counts else "     Active Tracks:           0")
    print(f"     LiDAR-Camera Fused Ties: {fused_associations_count}")
    print(f"     Minimum TTC Observed:    {min_ttc_observed:.2f} s" if min_ttc_observed != float("inf") else "     Minimum TTC Observed:    None (>100s)")
    print(f"     Risk State Breakdown:    {risk_level_durations}")
    print("=" * 70)

    # 6. Save Markdown & JSON Reports
    report_md_path = os.path.join(save_dir, f"eval_report_{timestamp_str}.md")
    metrics_json_path = os.path.join(save_dir, f"metrics_{timestamp_str}.json")

    metrics_dict = {
        "timestamp": timestamp_str,
        "duration_sec": total_duration,
        "frames_processed": frame_count,
        "effective_fps": avg_fps,
        "camera_source": camera_source,
        "model_name": detector.active_model_name,
        "video_recorded_path": output_video_path,
        "latency_ms": {
            "mean": avg_infer_ms,
            "p95": p95_infer_ms,
            "min": min_infer_ms,
            "max": max_infer_ms
        },
        "detections": {
            "vehicles_total": detection_counts["vehicle"],
            "vehicle_subclasses": vehicle_subclasses,
            "workers_total": detection_counts["worker"],
            "worker_ppe_verified": worker_ppe_counts["verified"],
            "ppe_compliance_pct": ppe_compliance_pct,
            "mean_confidence": float(np.mean(confidences)) if confidences else 0.0
        },
        "lidar": {
            "packets_received": lidar_packets_seen,
            "sample_rate_hz": lidar_rate_hz,
            "min_distance_m": float(min(lidar_distances)) if lidar_distances else None,
            "max_distance_m": float(max(lidar_distances)) if lidar_distances else None
        },
        "worker_tag": {
            "packets_received": worker_packets_seen,
            "sample_rate_hz": worker_rate_hz,
            "motion_states": worker_states,
            "mean_svm_g": float(np.mean(worker_svms)) if worker_svms else None
        },
        "fusion_and_risk": {
            "fused_associations": fused_associations_count,
            "min_ttc_sec": float(min_ttc_observed) if min_ttc_observed != float("inf") else None,
            "risk_distribution": risk_level_durations
        }
    }

    with open(metrics_json_path, "w") as jf:
        json.dump(metrics_dict, jf, indent=2)

    # Markdown report
    with open(report_md_path, "w") as mf:
        mf.write(f"# Real-Time Safety Pole System - Model Evaluation Report\n\n")
        mf.write(f"**Date/Time:** {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  \n")
        mf.write(f"**Camera Source:** `{camera_source}`  \n")
        mf.write(f"**Active Model:** `{detector.active_model_name}`  \n")
        mf.write(f"**Duration:** {total_duration:.2f} seconds ({frame_count} frames)  \n")
        if output_video_path:
            mf.write(f"**Recorded Video:** `{output_video_path}`  \n\n")
        else:
            mf.write("\n")
        
        mf.write("## 1. Computer Vision & Inference Latency\n")
        mf.write(f"| Metric | Value |\n|---|---|\n")
        mf.write(f"| **Effective FPS** | `{avg_fps:.2f} FPS` |\n")
        mf.write(f"| **Mean Inference Time** | `{avg_infer_ms:.2f} ms` |\n")
        mf.write(f"| **95th Percentile Latency** | `{p95_infer_ms:.2f} ms` |\n")
        mf.write(f"| **Min / Max Latency** | `{min_infer_ms:.2f} ms / {max_infer_ms:.2f} ms` |\n\n")

        mf.write("## 2. Object Detection & PPE Verification\n")
        mf.write(f"| Metric | Value |\n|---|---|\n")
        mf.write(f"| **Vehicles Detected** | `{detection_counts['vehicle']}` |\n")
        mf.write(f"| **Vehicle Breakdown** | `{vehicle_subclasses}` |\n")
        mf.write(f"| **Workers Detected** | `{detection_counts['worker']}` |\n")
        mf.write(f"| **PPE Compliance Rate** | `{ppe_compliance_pct:.1f}%` |\n")
        mf.write(f"| **Mean Detection Confidence** | `{metrics_dict['detections']['mean_confidence']*100:.1f}%` |\n\n")

        mf.write("## 3. Hardware Sensors Performance\n")
        mf.write(f"| Sensor | Packet Count | Frequency | Status |\n|---|---|---|---|\n")
        mf.write(f"| **TSD20 LiDAR (UDP 5006)** | {lidar_packets_seen} | {lidar_rate_hz:.1f} Hz | {'PASS (Target ~50Hz)' if lidar_rate_hz > 20 else 'CHECK WI-FI'} |\n")
        mf.write(f"| **ESP32-S3 Worker (UDP 5005)** | {worker_packets_seen} | {worker_rate_hz:.1f} Hz | {'PASS (Target ~25Hz)' if worker_rate_hz > 10 else 'CHECK WI-FI'} |\n\n")

        mf.write("## 4. Multi-Sensor Fusion & Collision Risk Assessment\n")
        mf.write(f"- **Camera-LiDAR Spatial Associations:** `{fused_associations_count}`\n")
        mf.write(f"- **Minimum TTC Recorded:** `{min_ttc_observed:.2f}s`\n")
        mf.write(f"- **Risk Level Distribution:** `{risk_level_durations}`\n")
        if output_video_path:
            mf.write(f"- **Full Annotated Evaluation Video:** Saved to `{output_video_path}`\n")
        if save_snapshots:
            mf.write(f"- **Inspection Snapshots:** Saved `{saved_snapshot_count}` images to `{snapshot_dir}`\n")

    print(f"\n[DONE] Evaluation Markdown Report: {report_md_path}")
    print(f"[DONE] Raw JSON Metrics:            {metrics_json_path}")
    if output_video_path:
        print(f"[DONE] Recorded Output Video:       {output_video_path}")
    if save_snapshots:
        print(f"[DONE] Annotated Snapshots:          {snapshot_dir} ({saved_snapshot_count} images)")
    print("=" * 70)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate Safety Pole Model on Real-Time Live Data with Video Output")
    parser.add_argument("--camera", type=str, default="picam2",
                        help="Camera source: 'picam2' (Pi CSI), '0' (USB Webcam), or video file path")
    parser.add_argument("--model", type=str, default=CONFIG.cv.MODEL_PATH,
                        help="Path or name of YOLO model weights (.pt file)")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Evaluation duration in seconds (0 for continuous until Ctrl+C)")
    parser.add_argument("--display", action="store_true",
                        help="Display real-time interactive video window with bounding boxes & threat banners")
    parser.add_argument("--record", action="store_true",
                        help="Record and save annotated evaluation video to an .mp4 file")
    parser.add_argument("--save-dir", type=str, default="eval_results",
                        help="Directory to save evaluation reports, videos, and snapshots")
    parser.add_argument("--no-snapshots", action="store_true",
                        help="Disable saving annotated frame snapshots")
    args = parser.parse_args()

    run_realtime_evaluation(
        camera_source=args.camera,
        model_path=args.model,
        duration_sec=args.duration,
        save_dir=args.save_dir,
        save_snapshots=not args.no_snapshots,
        display=args.display,
        record=args.record
    )
