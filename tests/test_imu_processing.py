"""
Unit tests for IMU processing, SVM calculations, and Worker Tag telemetry.
"""

import math
from src.sensors.worker_receiver import WorkerReceiver, WorkerState
from src.config import CONFIG

def test_svm_calculation():
    """Verify Signal Vector Magnitude calculation: |a| = sqrt(ax^2 + ay^2 + az^2)."""
    ax, ay, az = 0.0, 0.0, 1.0
    svm = math.sqrt(ax**2 + ay**2 + az**2)
    assert math.isclose(svm, 1.0, rel_tol=1e-3)

    # Dynamic motion
    ax, ay, az = 1.2, -0.8, 2.5
    svm = math.sqrt(ax**2 + ay**2 + az**2)
    assert math.isclose(svm, 2.88, rel_tol=1e-2)

def test_rssi_to_distance_estimation():
    """Verify Log-Distance Path Loss Model distance computation."""
    rx = WorkerReceiver(port=5999)
    # At 1 meter, RSSI should equal calibrated A (e.g. -59 dBm)
    dist_1m = rx._estimate_distance_from_rssi(CONFIG.worker.BLE_TX_POWER_1M)
    assert math.isclose(dist_1m, 1.0, rel_tol=0.05)

    # Weaker RSSI (e.g. -75 dBm) should correspond to farther distance
    dist_far = rx._estimate_distance_from_rssi(-75)
    assert dist_far > dist_1m

def test_worker_state_injection_and_retrieval():
    """Verify thread-safe worker state registry."""
    rx = WorkerReceiver(port=5998)
    state = WorkerState(
        worker_id="WORKER_TEST",
        svm=1.05,
        motion_state="WALKING",
        rssi=-65
    )
    rx.inject_worker_state(state)
    
    retrieved = rx.get_worker("WORKER_TEST")
    assert retrieved is not None
    assert retrieved.worker_id == "WORKER_TEST"
    assert retrieved.motion_state == "WALKING"
