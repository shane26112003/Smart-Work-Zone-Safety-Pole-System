#!/usr/bin/env python3
"""
Generates a lightweight, compact (under 50 KB) benchmark trajectory dataset
for evaluating vehicle trajectory prediction models without downloading gigabytes of data.

Contains 20 realistic highway & work-zone vehicle trajectories sampled at 10 Hz (5.0s per vehicle).
Output: sample_data/vehicle_trajectories_benchmark.csv
"""

import os
import csv
import math

SAMPLE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sample_data")
CSV_PATH = os.path.join(SAMPLE_DIR, "vehicle_trajectories_benchmark.csv")


def generate_benchmark_csv():
    os.makedirs(SAMPLE_DIR, exist_ok=True)
    rows = []

    # 20 Scenarios covering the full spectrum of road and work-zone dynamics
    scenarios = [
        # Track 1-4: Constant Speed Approaches
        {"id": "VEH_01", "name": "Highway Cruise (60 km/h)", "v0": 16.67, "x0": 3.5, "y0": 90.0, "ax": 0.0, "ay": 0.0},
        {"id": "VEH_02", "name": "Highway Cruise (80 km/h)", "v0": 22.22, "x0": 3.5, "y0": 120.0, "ax": 0.0, "ay": 0.0},
        {"id": "VEH_03", "name": "Highway Cruise (90 km/h)", "v0": 25.00, "x0": 3.5, "y0": 130.0, "ax": 0.0, "ay": 0.0},
        {"id": "VEH_04", "name": "Fast Lane Cruise (110 km/h)", "v0": 30.55, "x0": 7.0, "y0": 160.0, "ax": 0.0, "ay": 0.0},

        # Track 5-8: Deceleration / Work-Zone Braking
        {"id": "VEH_05", "name": "Gentle Work-Zone Braking (-1.8 m/s^2)", "v0": 22.22, "x0": 3.5, "y0": 110.0, "ax": 0.0, "ay": 1.8},
        {"id": "VEH_06", "name": "Standard Braking (-3.2 m/s^2)", "v0": 22.22, "x0": 3.5, "y0": 100.0, "ax": 0.0, "ay": 3.2},
        {"id": "VEH_07", "name": "Hard Work-Zone Braking (-5.0 m/s^2)", "v0": 22.22, "x0": 3.5, "y0": 95.0, "ax": 0.0, "ay": 5.0},
        {"id": "VEH_08", "name": "Emergency Braking (-7.0 m/s^2)", "v0": 25.00, "x0": 3.5, "y0": 115.0, "ax": 0.0, "ay": 7.0},

        # Track 9-12: Lane Changes & Swerves
        {"id": "VEH_09", "name": "Smooth Left Lane Change", "v0": 19.44, "x0": 7.0, "y0": 100.0, "ax": -1.2, "ay": 0.0, "type": "lane_change"},
        {"id": "VEH_10", "name": "Smooth Right Lane Change", "v0": 19.44, "x0": 0.5, "y0": 100.0, "ax": 1.2, "ay": 0.0, "type": "lane_change"},
        {"id": "VEH_11", "name": "Work-Zone Taper Swerve Left", "v0": 16.67, "x0": 3.5, "y0": 85.0, "ax": -1.5, "ay": 0.5, "type": "lane_change"},
        {"id": "VEH_12", "name": "Rapid Evasive Swerve Right", "v0": 18.00, "x0": 3.0, "y0": 90.0, "ax": 2.2, "ay": 1.0, "type": "lane_change"},

        # Track 13-16: Errant Vehicles / Barrier & Cone Encroachment
        {"id": "VEH_13", "name": "Drift towards Cones (Shallow)", "v0": 15.00, "x0": 2.5, "y0": 75.0, "ax": -0.8, "ay": 0.0, "type": "drift"},
        {"id": "VEH_14", "name": "Direct Work-Zone Encroachment", "v0": 16.67, "x0": 2.0, "y0": 80.0, "ax": -1.6, "ay": 0.0, "type": "drift"},
        {"id": "VEH_15", "name": "Distracted Driver Incline Drift", "v0": 20.00, "x0": 3.0, "y0": 100.0, "ax": -1.1, "ay": 0.0, "type": "drift"},
        {"id": "VEH_16", "name": "Aggressive Work-Zone Breach", "v0": 22.22, "x0": 2.5, "y0": 110.0, "ax": -2.0, "ay": 0.0, "type": "drift"},

        # Track 17-20: Stop-and-Go Congestion / Low Speed Queuing
        {"id": "VEH_17", "name": "Work-Zone Crawl (20 km/h)", "v0": 5.55, "x0": 3.5, "y0": 30.0, "ax": 0.0, "ay": 0.0},
        {"id": "VEH_18", "name": "Stop-and-Go Creep (10 km/h)", "v0": 2.78, "x0": 3.5, "y0": 20.0, "ax": 0.0, "ay": 0.0},
        {"id": "VEH_19", "name": "Work-Zone Exit Acceleration (+2.0 m/s^2)", "v0": 5.00, "x0": 3.5, "y0": 35.0, "ax": 0.0, "ay": -2.0},
        {"id": "VEH_20", "name": "Curved Taper Approach", "v0": 14.00, "x0": 4.0, "y0": 70.0, "ax": -0.7, "ay": 0.4, "type": "curve"}
    ]

    total_steps = 50  # 5.0 seconds at 10 Hz (dt = 0.1s)
    dt = 0.1

    for sc in scenarios:
        v_id = sc["id"]
        x = sc["x0"]
        y = sc["y0"]
        v_long = sc["v0"]  # forward longitudinal speed towards pole
        ax_lat = sc.get("ax", 0.0)
        ay_decel = sc.get("ay", 0.0) # deceleration
        s_type = sc.get("type", "normal")

        for step in range(total_steps):
            t = round(step * dt, 2)
            
            # Position update
            if s_type == "lane_change":
                progress = min(1.0, t / 3.0)
                smooth_shift = (3 * progress**2 - 2 * progress**3)
                cur_x = sc["x0"] + ax_lat * smooth_shift
            elif s_type in ["drift", "curve"]:
                cur_x = sc["x0"] + (ax_lat * t)
            else:
                cur_x = sc["x0"]

            # Longitudinal deceleration dynamics
            if ay_decel > 0 and t > 1.5: # brakes applied after 1.5s
                t_brake = t - 1.5
                cur_v = max(1.0, v_long - ay_decel * t_brake)
                cur_y = y - (cur_v * dt)
            elif ay_decel < 0: # acceleration
                cur_v = v_long + (-ay_decel * t)
                cur_y = y - (cur_v * dt)
            else:
                cur_v = v_long
                cur_y = y - (v_long * dt)

            y = cur_y
            speed_kmh = cur_v * 3.6

            rows.append({
                "track_id": v_id,
                "timestamp": f"{t:.2f}",
                "x": f"{cur_x:.3f}",
                "y": f"{cur_y:.3f}",
                "speed_kmh": f"{speed_kmh:.1f}",
                "scenario": sc["name"]
            })

    with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["track_id", "timestamp", "x", "y", "speed_kmh", "scenario"])
        writer.writeheader()
        writer.writerows(rows)

    file_size_kb = os.path.getsize(CSV_PATH) / 1024
    print(f"[SUCCESS] Generated benchmark dataset: {CSV_PATH}")
    print(f"          Total Sequences: {len(scenarios)} vehicles (1,000 trajectory points)")
    print(f"          File Size:       {file_size_kb:.1f} KB (Ultra lightweight!)")


if __name__ == "__main__":
    generate_benchmark_csv()
