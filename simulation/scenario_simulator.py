"""
Interactive Multi-Agent Scenario Simulator.
Simulates road traffic, worker trajectories, LiDAR ranging, and ESP32 IMU dynamics.
Provides instant validation and demonstrations for the safety system.
"""

import time
import math
import threading
from typing import Optional, Dict, Any
import logging

from src.config import CONFIG
from src.sensors.lidar_tsd20 import LidarTSD20
from src.sensors.worker_receiver import WorkerReceiver, WorkerState
from src.sensors.camera_manager import CameraManager

logger = logging.getLogger("ScenarioSimulator")


class ScenarioSimulator:
    """
    Coordinates dynamic multi-agent scenario simulations.
    Injects synthetic sensor signals into Camera, LiDAR, and ESP32 telemetry pipelines.
    """

    def __init__(self, 
                 lidar: LidarTSD20, 
                 worker_rx: WorkerReceiver, 
                 camera_mgr: CameraManager):
        self.lidar = lidar
        self.worker_rx = worker_rx
        self.camera_mgr = camera_mgr

        self.running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Current scenario: "normal", "distracted_driver", "worker_incursion", "worker_fall"
        self.active_scenario = "normal"
        self.scenario_start_time = time.time()

        # Actor Kinematic States
        # Vehicle: Approaching along road (Positive Y towards Pole Y=0)
        self.veh_x = 3.5          # Center of traffic lane
        self.veh_y = 55.0         # 55 meters away
        self.veh_vx = 0.0         # Lateral speed m/s
        self.veh_vy = -12.5       # ~45 km/h approaching speed
        
        # Worker: In work zone
        self.wrk_x = -3.5         # Safely inside work zone
        self.wrk_y = 6.0          # 6 meters longitudinally from pole
        self.wrk_vx = 0.0
        self.wrk_vy = 0.0
        self.wrk_motion_state = "STATIC"
        self.wrk_svm = 1.0

    def start(self):
        """Starts simulator thread."""
        if self.running:
            return
        self.running = True
        self.scenario_start_time = time.time()
        self._thread = threading.Thread(target=self._sim_loop, daemon=True, name="ScenarioSimulator")
        self._thread.start()
        logger.info(f"ScenarioSimulator started with scenario: '{self.active_scenario}'")

    def stop(self):
        """Stops simulator."""
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logger.info("ScenarioSimulator stopped.")

    def set_scenario(self, scenario_name: str):
        """Switches active scenario dynamically."""
        with self._lock:
            self.active_scenario = scenario_name
            self.scenario_start_time = time.time()
            
            # Reset actor states based on scenario
            if scenario_name == "normal":
                self.veh_x = 3.5
                self.veh_y = 55.0
                self.veh_vx = 0.0
                self.veh_vy = -11.0
                self.wrk_x = -3.5
                self.wrk_y = 6.0
                self.wrk_vx = 0.0
                self.wrk_motion_state = "STATIC"
                self.wrk_svm = 1.0

            elif scenario_name == "distracted_driver":
                self.veh_x = 3.0
                self.veh_y = 50.0
                self.veh_vx = -1.2   # Drifts sharply left across cone line towards worker!
                self.veh_vy = -13.0  # 47 km/h
                self.wrk_x = -2.0
                self.wrk_y = 5.0
                self.wrk_vx = 0.0
                self.wrk_motion_state = "STATIC"
                self.wrk_svm = 1.02

            elif scenario_name == "worker_incursion":
                self.veh_x = 2.5
                self.veh_y = 45.0
                self.veh_vx = 0.0
                self.veh_vy = -11.5
                self.wrk_x = -3.0
                self.wrk_y = 7.0
                self.wrk_vx = 1.1    # Worker walks rapidly right into traffic lane!
                self.wrk_motion_state = "WALKING"
                self.wrk_svm = 1.35

            elif scenario_name == "worker_fall":
                self.veh_x = 2.0
                self.veh_y = 40.0
                self.veh_vx = 0.0
                self.veh_vy = -10.0
                self.wrk_x = -0.6    # Right on boundary line
                self.wrk_y = 5.0
                self.wrk_vx = 0.0
                self.wrk_motion_state = "FALL_DETECTED"
                self.wrk_svm = 3.65

        logger.info(f"Switched scenario to: {scenario_name}")

    def _sim_loop(self):
        dt = 0.04 # 25 Hz simulation step

        while self.running:
            t0 = time.time()
            with self._lock:
                # Update kinematics
                self.veh_x += self.veh_vx * dt
                self.veh_y += self.veh_vy * dt
                self.wrk_x += self.wrk_vx * dt
                self.wrk_y += self.wrk_vy * dt

                # Reset vehicle if it has passed the pole
                if self.veh_y <= 1.0:
                    self.veh_y = 55.0
                    if self.active_scenario == "distracted_driver":
                        self.veh_x = 3.5
                    elif self.active_scenario == "worker_incursion":
                        self.wrk_x = -3.0

                # In worker incursion scenario, clamp worker once they enter road
                if self.active_scenario == "worker_incursion" and self.wrk_x >= 1.5:
                    self.wrk_vx = 0.0
                    self.wrk_motion_state = "STATIC"
                    self.wrk_svm = 1.0

                # 1. Feed Synthetic Camera Actors
                veh_speed = math.sqrt(self.veh_vx**2 + self.veh_vy**2)
                self.camera_mgr.update_simulation_actors(self.veh_y, veh_speed, self.wrk_x, self.wrk_y)

                # 2. Feed Simulated TSD20 LiDAR distance
                # TSD20 measures vehicles approaching along road
                lidar_dist = max(0.2, self.veh_y)
                lidar_valid = (lidar_dist <= CONFIG.lidar.MAX_RANGE_M)
                self.lidar.inject_reading(
                    distance_m=lidar_dist if lidar_valid else 20.0,
                    range_rate_mps=self.veh_vy,
                    valid=lidar_valid
                )

                # 3. Feed Simulated ESP32 Worker Telemetry
                dist_to_pole = math.sqrt(self.wrk_x**2 + self.wrk_y**2)
                # Compute RSSI corresponding to this distance
                # RSSI = A - 10 * n * log10(d)
                sim_rssi = int(CONFIG.worker.BLE_TX_POWER_1M - 10.0 * CONFIG.worker.BLE_PATH_LOSS_EXPONENT * math.log10(max(0.5, dist_to_pole)))

                mock_worker = WorkerState(
                    worker_id=CONFIG.worker.DEFAULT_WORKER_ID,
                    ax=0.05 if self.wrk_motion_state == "STATIC" else 0.45,
                    ay=0.02,
                    az=0.98 if self.wrk_motion_state != "FALL_DETECTED" else 0.12,
                    svm=self.wrk_svm,
                    pitch=-2.0,
                    roll=5.0 if self.wrk_motion_state != "FALL_DETECTED" else 85.0,
                    motion_state=self.wrk_motion_state,
                    battery_v=4.12,
                    rssi=sim_rssi,
                    estimated_dist_m=dist_to_pole,
                    last_seen=time.time(),
                    is_active=True,
                    x=self.wrk_x,
                    y=self.wrk_y,
                    vx=self.wrk_vx,
                    vy=self.wrk_vy
                )
                self.worker_rx.inject_worker_state(mock_worker)

            elapsed = time.time() - t0
            time.sleep(max(0.005, dt - elapsed))
