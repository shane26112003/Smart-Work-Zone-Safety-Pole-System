"""
Collision-Risk Assessment Engine.
Evaluates spatial-temporal relationships between vehicles and construction workers.
Computes:
 - Euclidean distance & Relative velocity vectors
 - Closest Point of Approach (CPA) distance and time
 - Time-To-Collision (TTC)
 - Trajectory overlap / intersection cone
 - Worker IMU vulnerability factors (FALL_DETECTED, IMPACT, RUNNING towards road)
 - LiDAR-confirmed closure rates
Classifies risk state: SAFE -> CAUTION -> HIGH RISK -> CRITICAL.
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import logging

from ..config import CONFIG
from ..fusion.ekf_fusion import FusedEntity

logger = logging.getLogger("RiskEngine")

@dataclass
class PairRisk:
    """Risk evaluation for a specific Vehicle-Worker pair."""
    vehicle_id: str
    worker_id: str
    risk_level: str                 # "SAFE", "CAUTION", "HIGH RISK", "CRITICAL"
    distance_m: float
    cpa_distance_m: float           # Closest Point of Approach miss-distance
    ttc_sec: float                  # Time-To-Collision (inf if divergent)
    relative_speed_mps: float
    vehicle_speed_kmh: float
    worker_motion_state: str
    is_trajectory_converging: bool
    is_worker_in_hazard_zone: bool
    reasons: List[str] = field(default_factory=list)


@dataclass
class EvasiveGuidance:
    """Actionable evasion directive tailored to a specific worker."""
    target_worker_id: str
    action_code: str                # "CLEAR_ROADWAY_LEFT", "RETREAT_DEEPER", "STAY_DOWN", "EVACUATE_LEFT", "HOLD_POSITION"
    directive_text: str             # Human-readable instruction (e.g., "STEP 2M LEFT INTO SAFE ZONE")
    escape_vector: Tuple[float, float] # (dx, dy) direction vector to escape hazard
    target_safe_pos: Tuple[float, float] # (x, y) recommended metric coordinate
    urgency: str                    # "LOW", "MODERATE", "URGENT", "EMERGENCY"
    haptic_pattern: str             # "NONE", "PULSE_SLOW", "PULSE_RAPID", "CONTINUOUS_SOLID"
    ttc_sec: float


@dataclass
class RiskAssessment:
    """Overall work-zone safety status."""
    overall_level: str = "SAFE"     # "SAFE", "CAUTION", "HIGH RISK", "CRITICAL"
    min_ttc_sec: float = 99.0
    min_distance_m: float = 99.0
    critical_pair: Optional[Tuple[str, str]] = None
    warning_active: bool = False
    warning_type: str = "NONE"      # "NONE", "VISUAL_CAUTION", "AUDIO_HIGH", "EMERGENCY_SIREN"
    active_pairs: List[PairRisk] = field(default_factory=list)
    guidance: Optional[EvasiveGuidance] = None
    timestamp: float = field(default_factory=time.time)


class RiskEngine:
    """
    Evaluates dynamic risks across all detected vehicles and workers.
    Applies multi-sensor verification to minimize false alarms while guaranteeing
    instant reaction to genuine collision threats.
    """

    def __init__(self):
        self.last_assessment = RiskAssessment()

    def evaluate(self, entities: List[FusedEntity]) -> RiskAssessment:
        """
        Runs complete risk assessment across all active entities.
        """
        now = time.time()
        vehicles = [e for e in entities if e.entity_type == "vehicle"]
        workers = [e for e in entities if e.entity_type == "worker"]

        pair_risks: List[PairRisk] = []
        overall_level = "SAFE"
        min_ttc = 99.0
        min_dist = 99.0
        critical_pair = None

        # Level hierarchy for escalation
        level_priority = {"SAFE": 0, "CAUTION": 1, "HIGH RISK": 2, "CRITICAL": 3}
        current_priority = 0

        # Evaluate every Vehicle-Worker combination
        for veh in vehicles:
            for wrk in workers:
                p_risk = self._evaluate_pair(veh, wrk)
                pair_risks.append(p_risk)

                if p_risk.distance_m < min_dist:
                    min_dist = p_risk.distance_m

                if p_risk.ttc_sec < min_ttc:
                    min_ttc = p_risk.ttc_sec

                if level_priority[p_risk.risk_level] > current_priority:
                    current_priority = level_priority[p_risk.risk_level]
                    overall_level = p_risk.risk_level
                    critical_pair = (p_risk.vehicle_id, p_risk.worker_id)

        # Worker isolated hazards (e.g. fallen worker in roadway even without immediate car)
        for wrk in workers:
            m_state = wrk.worker_state.motion_state if wrk.worker_state else "STATIC"
            in_road = (wrk.x >= CONFIG.geometry.HAZARD_LINE_X - 0.5)

            if m_state == "FALL_DETECTED":
                fall_level = "CRITICAL" if in_road or len(vehicles) > 0 else "HIGH RISK"
                if level_priority[fall_level] > current_priority:
                    current_priority = level_priority[fall_level]
                    overall_level = fall_level
                    critical_pair = ("ENVIRONMENT", wrk.entity_id)

        # Determine warning actuator state
        warning_active = (current_priority >= level_priority["CAUTION"])
        warning_type = "NONE"
        if overall_level == "CAUTION":
            warning_type = "VISUAL_CAUTION"
        elif overall_level == "HIGH RISK":
            warning_type = "AUDIO_HIGH"
        elif overall_level == "CRITICAL":
            warning_type = "EMERGENCY_SIREN"

        # Generate actionable evasive guidance for targeted worker
        guidance = self._generate_guidance(vehicles, workers, critical_pair, overall_level, min_ttc)

        assessment = RiskAssessment(
            overall_level=overall_level,
            min_ttc_sec=round(min_ttc, 2) if min_ttc < 90.0 else 99.0,
            min_distance_m=round(min_dist, 2) if min_dist < 90.0 else 99.0,
            critical_pair=critical_pair,
            warning_active=warning_active,
            warning_type=warning_type,
            active_pairs=pair_risks,
            guidance=guidance,
            timestamp=now
        )
        self.last_assessment = assessment
        return assessment

    def _generate_guidance(self, vehicles: List[FusedEntity], workers: List[FusedEntity], 
                           critical_pair: Optional[Tuple[str, str]], 
                           overall_level: str, min_ttc: float) -> Optional[EvasiveGuidance]:
        """
        Determines the optimal evasive action directive for the specific worker at risk.
        """
        if not workers or overall_level == "SAFE":
            target_id = workers[0].entity_id if workers else "ALL_WORKERS"
            return EvasiveGuidance(
                target_worker_id=target_id,
                action_code="HOLD_POSITION",
                directive_text="Work zone secure. Maintain standard perimeter awareness.",
                escape_vector=(0.0, 0.0),
                target_safe_pos=(-3.0, 5.0),
                urgency="LOW",
                haptic_pattern="NONE",
                ttc_sec=99.0
            )

        # Identify the critical worker
        target_wrk = None
        target_veh = None
        if critical_pair:
            target_wrk = next((w for w in workers if w.entity_id == critical_pair[1]), None)
            target_veh = next((v for v in vehicles if v.entity_id == critical_pair[0]), None)
        if not target_wrk:
            target_wrk = workers[0]

        # 1. Fall Condition: Worker cannot run -> Command to protect vitals & alert crew
        if target_wrk.worker_state and target_wrk.worker_state.motion_state == "FALL_DETECTED":
            return EvasiveGuidance(
                target_worker_id=target_wrk.entity_id,
                action_code="STAY_DOWN_PROTECT_HEAD",
                directive_text="WORKER DOWN! STAY DOWN & COVER HEAD. EMERGENCY SIREN ACTIVATED.",
                escape_vector=(0.0, 0.0),
                target_safe_pos=(target_wrk.x, target_wrk.y),
                urgency="EMERGENCY",
                haptic_pattern="CONTINUOUS_SOLID",
                ttc_sec=round(min_ttc, 1)
            )

        # 2. Road Incursion: Worker is past safety cones in active traffic lane (X >= -0.3m)
        if target_wrk.x >= CONFIG.geometry.HAZARD_LINE_X - 0.3:
            return EvasiveGuidance(
                target_worker_id=target_wrk.entity_id,
                action_code="CLEAR_ROADWAY_LEFT",
                directive_text="CLEAR ROADWAY! STEP 2.5M LEFT INTO WORK ZONE IMMEDIATELY.",
                escape_vector=(-2.5, 0.0),
                target_safe_pos=(CONFIG.geometry.WORKZONE_X_MAX - 1.5, target_wrk.y),
                urgency="EMERGENCY" if overall_level == "CRITICAL" else "URGENT",
                haptic_pattern="PULSE_RAPID",
                ttc_sec=round(min_ttc, 1)
            )

        # 3. Vehicle Breaching Cones: Approaching car has lateral velocity towards work zone
        if target_veh and target_veh.vx < -0.3:
            return EvasiveGuidance(
                target_worker_id=target_wrk.entity_id,
                action_code="RETREAT_DEEPER_INTO_ZONE",
                directive_text=f"VEHICLE DRIFTING IN! RETREAT 3M DEEPER INTO ZONE (TTC: {min_ttc:.1f}s)",
                escape_vector=(-3.0, 0.0),
                target_safe_pos=(max(CONFIG.geometry.WORKZONE_X_MIN, target_wrk.x - 3.0), target_wrk.y),
                urgency="EMERGENCY" if overall_level == "CRITICAL" else "URGENT",
                haptic_pattern="PULSE_RAPID",
                ttc_sec=round(min_ttc, 1)
            )

        # 4. Imminent Collision on Trajectory Intersection
        if overall_level in ["CRITICAL", "HIGH RISK"]:
            return EvasiveGuidance(
                target_worker_id=target_wrk.entity_id,
                action_code="EVACUATE_LEFT",
                directive_text=f"COLLISION THREAT! EVACUATE 2M LEFT AWAY FROM BOUNDARY (TTC: {min_ttc:.1f}s)",
                escape_vector=(-2.0, 0.0),
                target_safe_pos=(target_wrk.x - 2.0, target_wrk.y),
                urgency="URGENT" if overall_level == "HIGH RISK" else "EMERGENCY",
                haptic_pattern="PULSE_RAPID",
                ttc_sec=round(min_ttc, 1)
            )

        # 5. Caution State: Approaching traffic
        return EvasiveGuidance(
            target_worker_id=target_wrk.entity_id,
            action_code="CAUTION_HOLD_BOUNDARY",
            directive_text="CAUTION: Approaching vehicle detected. Hold boundary line.",
            escape_vector=(-1.0, 0.0),
            target_safe_pos=(target_wrk.x - 1.0, target_wrk.y),
            urgency="MODERATE",
            haptic_pattern="PULSE_SLOW",
            ttc_sec=round(min_ttc, 1)
        )

    def _evaluate_pair(self, veh: FusedEntity, wrk: FusedEntity) -> PairRisk:
        """Computes kinematics, CPA, TTC, and risk score for a single vehicle-worker pair."""
        reasons = []

        # Relative position vector: delta_p = p_veh - p_wrk
        dx = veh.x - wrk.x
        dy = veh.y - wrk.y
        dist = math.sqrt(dx*dx + dy*dy)

        # Relative velocity vector: delta_v = v_veh - v_wrk
        dvx = veh.vx - wrk.vx
        dvy = veh.vy - wrk.vy
        rel_speed = math.sqrt(dvx*dvx + dvy*dvy)

        # Closest Point of Approach (CPA) calculation
        # t_cpa = - (delta_p . delta_v) / |delta_v|^2
        dot_product = dx * dvx + dy * dvy
        v_sq = dvx * dvx + dvy * dvy

        is_converging = False
        t_cpa = 99.0
        d_cpa = dist

        if v_sq > 0.01:
            # If dot_product < 0, distance is decreasing (converging)
            if dot_product < 0:
                is_converging = True
                t_cpa = -dot_product / v_sq
                
                # Position at CPA
                cpa_x = dx + dvx * t_cpa
                cpa_y = dy + dvy * t_cpa
                d_cpa = math.sqrt(cpa_x * cpa_x + cpa_y * cpa_y)

        # Time-To-Collision (TTC)
        ttc = t_cpa if (is_converging and d_cpa <= CONFIG.risk.CPA_COLLISION_DISTANCE_M) else 99.0

        # Worker context checks
        wrk_motion = wrk.worker_state.motion_state if wrk.worker_state else "STATIC"
        worker_in_hazard_zone = (wrk.x >= CONFIG.geometry.HAZARD_LINE_X - CONFIG.risk.HAZARD_LINE_BUFFER_M)
        vehicle_drifting_into_workzone = (veh.vx < -0.4 and veh.x < CONFIG.geometry.ROAD_LANE_X_MIN + 1.0)

        # Classification Logic
        risk_level = "SAFE"

        # 1. CRITICAL Conditions
        if (ttc <= CONFIG.risk.TTC_CRITICAL_SEC and d_cpa <= CONFIG.risk.CPA_COLLISION_DISTANCE_M):
            risk_level = "CRITICAL"
            reasons.append(f"Imminent trajectory collision (TTC: {ttc:.1f}s, Miss: {d_cpa:.1f}m)")
        elif dist <= CONFIG.risk.DIST_CRITICAL_M:
            risk_level = "CRITICAL"
            reasons.append(f"Extreme vehicle proximity ({dist:.1f}m)")
        elif (wrk_motion == "FALL_DETECTED" and is_converging and dist <= 30.0):
            risk_level = "CRITICAL" if dist <= 15.0 else "HIGH RISK"
            reasons.append("Worker immobilized by fall with approaching traffic")
        elif (vehicle_drifting_into_workzone and dist < CONFIG.risk.DIST_HIGH_RISK_M and veh.speed_kmh > 20):
            risk_level = "CRITICAL"
            reasons.append(f"Vehicle breached work-zone boundary at {veh.speed_kmh:.0f} km/h")

        # 2. HIGH RISK Conditions
        elif risk_level == "SAFE":
            if (ttc <= CONFIG.risk.TTC_HIGH_RISK_SEC and d_cpa <= 3.5):
                risk_level = "HIGH RISK"
                reasons.append(f"Converging trajectory (TTC: {ttc:.1f}s)")
            elif dist <= CONFIG.risk.DIST_HIGH_RISK_M:
                risk_level = "HIGH RISK"
                reasons.append(f"Vehicle within danger boundary ({dist:.1f}m)")
            elif vehicle_drifting_into_workzone and is_converging:
                risk_level = "HIGH RISK"
                reasons.append("Vehicle trajectory heading toward work zone")
            elif worker_in_hazard_zone and veh.speed_kmh > 35 and dist < 20.0:
                risk_level = "HIGH RISK"
                reasons.append(f"Worker at road boundary with approaching traffic ({veh.speed_kmh:.0f} km/h)")

        # 3. CAUTION Conditions
        elif risk_level == "SAFE":
            if (ttc <= CONFIG.risk.TTC_CAUTION_SEC and d_cpa <= 5.0):
                risk_level = "CAUTION"
                reasons.append(f"Approaching vehicle (TTC: {ttc:.1f}s)")
            elif dist <= CONFIG.risk.DIST_CAUTION_M:
                risk_level = "CAUTION"
                reasons.append(f"Vehicle approaching work zone ({dist:.1f}m)")
            elif worker_in_hazard_zone:
                risk_level = "CAUTION"
                reasons.append("Worker close to active road boundary")

        return PairRisk(
            vehicle_id=veh.entity_id,
            worker_id=wrk.entity_id,
            risk_level=risk_level,
            distance_m=round(dist, 2),
            cpa_distance_m=round(d_cpa, 2),
            ttc_sec=round(ttc, 2) if ttc < 90.0 else 99.0,
            relative_speed_mps=round(rel_speed, 2),
            vehicle_speed_kmh=veh.speed_kmh,
            worker_motion_state=wrk_motion,
            is_trajectory_converging=is_converging,
            is_worker_in_hazard_zone=worker_in_hazard_zone,
            reasons=reasons
        )
