"""
Automated Test Runner using Standard Python.
Executes all test functions across test suites and reports results.
"""

import sys
import os
import traceback
import time

# Ensure project root is in python path
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)


def run_suite():
    tests = [
        ("test_imu_processing", [
            "test_svm_calculation",
            "test_rssi_to_distance_estimation",
            "test_worker_state_injection_and_retrieval"
        ]),
        ("test_lidar_driver", [
            "test_lidar_injection_and_range_clamping",
            "test_lidar_json_packet_parsing"
        ]),
        ("test_ekf_fusion", [
            "test_bev_transform_consistency",
            "test_ekf_filter_prediction_and_update",
            "test_sensor_fusion_engine_association"
        ]),
        ("test_risk_engine", [
            "test_safe_traffic_condition",
            "test_critical_trajectory_collision",
            "test_worker_fall_vulnerability_multiplier",
            "test_evasive_guidance_generation"
        ]),
        ("test_cv_detector", [
            "test_detector_initialization",
            "test_four_channel_and_grayscale_sanitization",
            "test_clahe_contrast_enhancement",
            "test_worker_ppe_verification",
            "test_synthetic_scene_worker_vehicle_separation",
            "test_tracker_subclass_and_ppe_propagation"
        ]),
        ("test_system_pipeline", [
            "test_full_pipeline_step"
        ])
    ]

    total = 0
    passed = 0
    failed = 0

    print("=" * 65)
    print("RUNNING SMART WORK-ZONE SAFETY POLE TEST SUITE")
    print("=" * 65)

    for module_name, func_names in tests:
        print(f"\n[SUITE] {module_name}")
        try:
            mod = __import__(f"tests.{module_name}", fromlist=func_names)
        except Exception as e:
            print(f"  [FAIL] IMPORT ERROR {module_name}: {e}")
            traceback.print_exc()
            failed += len(func_names)
            total += len(func_names)
            continue

        for fname in func_names:
            total += 1
            fn = getattr(mod, fname, None)
            if not fn:
                print(f"  [FAIL] {fname}: Not found")
                failed += 1
                continue

            try:
                t0 = time.time()
                fn()
                dt = (time.time() - t0) * 1000
                print(f"  [PASS] {fname} ({dt:.1f}ms)")
                passed += 1
            except Exception as e:
                print(f"  [FAIL] {fname}: {e}")
                traceback.print_exc()
                failed += 1

    print("\n" + "=" * 65)
    print(f"RESULTS: {passed}/{total} PASSED, {failed} FAILED")
    print("=" * 65)

    if failed > 0:
        sys.exit(1)

if __name__ == "__main__":
    run_suite()
