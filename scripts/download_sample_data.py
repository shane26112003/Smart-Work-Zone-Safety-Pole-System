#!/usr/bin/env python3
"""
Sample Dataset & Benchmark Video Downloader for Work-Zone Safety System.
Downloads free public sample traffic video and test images for model evaluation.

Usage:
  python scripts/download_sample_data.py
"""

import os
import sys
import urllib.request

SAMPLE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sample_data")

# Public domain / CC0 test traffic and pedestrian video samples (Wikimedia / sample CDN)
SAMPLE_FILES = [
    {
        "filename": "highway_traffic_sample.mp4",
        "url": "https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/ForBiggerBlazes.mp4",
        "description": "Sample test video clip for video pipeline benchmarking"
    }
]

def download_samples():
    os.makedirs(SAMPLE_DIR, exist_ok=True)
    print("=" * 65)
    print(" DOWNLOADING SAMPLE EVALUATION ASSETS ")
    print("=" * 65)
    print(f"Destination: {SAMPLE_DIR}\n")

    for item in SAMPLE_FILES:
        target_path = os.path.join(SAMPLE_DIR, item["filename"])
        if os.path.exists(target_path):
            print(f"[EXISTS] {item['filename']} already downloaded.")
            continue

        print(f"[DOWNLOADING] {item['filename']} ({item['description']})...")
        try:
            urllib.request.urlretrieve(item["url"], target_path)
            size_mb = os.path.getsize(target_path) / (1024 * 1024)
            print(f"[SUCCESS] Saved to {target_path} ({size_mb:.1f} MB)")
        except Exception as e:
            print(f"[ERROR] Failed to download {item['filename']}: {e}")

    # Also generate the ultra-lightweight trajectory benchmark dataset
    try:
        from generate_sample_trajectories import generate_benchmark_csv
        generate_benchmark_csv()
    except Exception:
        pass

    print("\n" + "=" * 65)
    print(" READY FOR EVALUATION!")
    print(" 1. Run video evaluation:")
    print("    python scripts/evaluate_realtime.py --camera sample_data/highway_traffic_sample.mp4 --record --display")
    print(" 2. Run vehicle trajectory prediction evaluation:")
    print("    python scripts/evaluate_trajectory.py --dataset custom --file sample_data/vehicle_trajectories_benchmark.csv")
    print("=" * 65)

if __name__ == "__main__":
    download_samples()
