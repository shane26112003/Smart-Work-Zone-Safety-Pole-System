"""
Unit tests for Collision-Risk Assessment Engine (TTC, CPA, Risk Escalation).
"""

from src.risk.risk_engine import RiskEngine
from src.fusion.ekf_fusion import FusedEntity
from src.sensors.worker_receiver import WorkerState
from src.config import CONFIG

def test_safe_traffic_condition():
    """Vehicle passing safely in its own lane far away from worker."""
    engine = RiskEngine()

    veh = FusedEntity(
        entity_id="VEH_01",
        entity_type="vehicle",
        x=4.0, y=35.0, # in lane
        vx=0.0, vy=-10.0, # moving straight down road
        speed_mps=10.0, speed_kmh=36.0
    )

    wrk = FusedEntity(
        entity_id="WORKER_01",
        entity_type="worker",
        x=-4.0, y=5.0, # safe inside work zone
        vx=0.0, vy=0.0
    )

    assessment = engine.evaluate([veh, wrk])
    assert assessment.overall_level in ["SAFE", "CAUTION"]
    assert assessment.warning_type != "EMERGENCY_SIREN"

def test_critical_trajectory_collision():
    """Vehicle swerving directly towards worker with imminent collision."""
    engine = RiskEngine()

    veh = FusedEntity(
        entity_id="VEH_01",
        entity_type="vehicle",
        x=-1.5, y=10.0, # Drifting into work zone
        vx=-0.5, vy=-12.0, # 43 km/h heading toward worker
        speed_mps=12.0, speed_kmh=43.2
    )

    wrk = FusedEntity(
        entity_id="WORKER_01",
        entity_type="worker",
        x=-2.0, y=2.0, # in line of travel
        vx=0.0, vy=0.0
    )

    assessment = engine.evaluate([veh, wrk])
    assert assessment.overall_level == "CRITICAL"
    assert assessment.min_ttc_sec <= CONFIG.risk.TTC_CRITICAL_SEC
    assert assessment.warning_active is True
    assert assessment.warning_type == "EMERGENCY_SIREN"

def test_worker_fall_vulnerability_multiplier():
    """Worker falls down in proximity to approaching vehicle."""
    engine = RiskEngine()

    worker_state = WorkerState(
        worker_id="WORKER_01",
        motion_state="FALL_DETECTED",
        svm=3.5
    )

    wrk = FusedEntity(
        entity_id="WORKER_01",
        entity_type="worker",
        x=-0.5, y=4.0, # Near boundary line
        vx=0.0, vy=0.0,
        worker_state=worker_state
    )

    veh = FusedEntity(
        entity_id="VEH_01",
        entity_type="vehicle",
        x=2.5, y=18.0,
        vx=0.0, vy=-10.0,
        speed_mps=10.0, speed_kmh=36.0
    )

    assessment = engine.evaluate([veh, wrk])
    assert assessment.overall_level in ["HIGH RISK", "CRITICAL"]
    assert "Worker immobilized by fall" in "".join(assessment.active_pairs[0].reasons)

def test_evasive_guidance_generation():
    """Verify actionable evasive guidance calculation for specific worker."""
    engine = RiskEngine()

    # Scenario: Worker stepped across cone line into active road
    wrk = FusedEntity(
        entity_id="WORKER_01",
        entity_type="worker",
        x=0.5, y=5.0, # inside road lane
        vx=0.0, vy=0.0
    )

    veh = FusedEntity(
        entity_id="VEH_01",
        entity_type="vehicle",
        x=1.5, y=25.0,
        vx=0.0, vy=-12.0,
        speed_mps=12.0, speed_kmh=43.2
    )

    assessment = engine.evaluate([veh, wrk])
    assert assessment.guidance is not None
    assert assessment.guidance.target_worker_id == "WORKER_01"
    assert assessment.guidance.action_code == "CLEAR_ROADWAY_LEFT"
    assert "CLEAR ROADWAY" in assessment.guidance.directive_text
    assert assessment.guidance.escape_vector[0] < 0 # Escape towards left (work zone)
    assert assessment.guidance.urgency in ["URGENT", "EMERGENCY"]
    assert assessment.guidance.haptic_pattern == "PULSE_RAPID"
