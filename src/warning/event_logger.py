"""
Black-Box Event & Incident Logger.
Persists safety risk events, sensor telemetry snapshots, and alarm triggers
to SQLite database and JSONL black-box audit files.
"""

import os
import json
import time
import sqlite3
import logging
from typing import Optional

from ..config import CONFIG
from ..risk.risk_engine import RiskAssessment

logger = logging.getLogger("EventLogger")


class EventLogger:
    """
    Records safety pole telemetry, risk transitions, and collision alarms.
    """

    def __init__(self, log_dir: Optional[str] = None):
        self.log_dir = log_dir or CONFIG.LOG_DIR
        os.makedirs(self.log_dir, exist_ok=True)

        self.jsonl_path = os.path.join(self.log_dir, CONFIG.INCIDENT_LOG_FILE)
        self.db_path = os.path.join(self.log_dir, "incidents.db")

        self.last_logged_level = "SAFE"
        self._init_db()

    def _init_db(self):
        """Initializes SQLite database schema."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS incident_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL,
                    datetime_str TEXT,
                    risk_level TEXT,
                    min_ttc REAL,
                    min_distance REAL,
                    vehicle_id TEXT,
                    worker_id TEXT,
                    reasons TEXT,
                    warning_type TEXT
                )
            """)
            conn.commit()
            conn.close()
        except Exception as e:
            logger.error(f"Failed to initialize SQLite incident DB: {e}")

    def log_assessment(self, assessment: RiskAssessment):
        """
        Logs assessment if risk level changes or if an active hazard (HIGH RISK/CRITICAL) occurs.
        """
        # Only log on state transition or active hazard
        is_hazard = assessment.overall_level in ["HIGH RISK", "CRITICAL"]
        state_changed = (assessment.overall_level != self.last_logged_level)

        if not (state_changed or (is_hazard and (time.time() % 3.0 < 0.2))):
            return

        self.last_logged_level = assessment.overall_level
        now = time.time()
        dt_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))

        veh_id = assessment.critical_pair[0] if assessment.critical_pair else "N/A"
        wrk_id = assessment.critical_pair[1] if assessment.critical_pair else "N/A"
        
        all_reasons = []
        for p in assessment.active_pairs:
            if p.risk_level == assessment.overall_level:
                all_reasons.extend(p.reasons)
        reasons_str = "; ".join(all_reasons) if all_reasons else "Normal operations"

        record = {
            "timestamp": now,
            "datetime": dt_str,
            "risk_level": assessment.overall_level,
            "min_ttc_sec": assessment.min_ttc_sec,
            "min_distance_m": assessment.min_distance_m,
            "vehicle_id": veh_id,
            "worker_id": wrk_id,
            "warning_type": assessment.warning_type,
            "reasons": reasons_str
        }

        # 1. Append to JSONL
        try:
            with open(self.jsonl_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record) + "\n")
        except Exception as e:
            logger.debug(f"JSONL write error: {e}")

        # 2. Insert into SQLite
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO incident_events (
                    timestamp, datetime_str, risk_level, min_ttc, min_distance,
                    vehicle_id, worker_id, reasons, warning_type
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                now, dt_str, assessment.overall_level, assessment.min_ttc_sec, assessment.min_distance_m,
                veh_id, wrk_id, reasons_str, assessment.warning_type
            ))
            conn.commit()
            conn.close()
        except Exception as e:
            logger.debug(f"SQLite insert error: {e}")
