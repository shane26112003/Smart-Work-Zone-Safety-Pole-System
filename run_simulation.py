"""
Interactive Simulation Launcher for Smart Work-Zone Safety Pole System.
Starts the system in full simulation mode with synthetic camera streams,
TSD20 LiDAR ranging, ESP32 worker telemetry, and the real-time web dashboard.
"""

import sys
import os

from main import SafetyPoleSystem
from src.config import CONFIG

if __name__ == "__main__":
    print("=" * 70)
    print("STARTING SMART WORK-ZONE SAFETY POLE SYSTEM [SIMULATION MODE]")
    print(f"Web Dashboard will be available at: http://localhost:{CONFIG.network.DASHBOARD_PORT}")
    print("=" * 70)

    system = SafetyPoleSystem(mode="simulation")
    try:
        system.start()
    except KeyboardInterrupt:
        system.stop()
        sys.exit(0)
