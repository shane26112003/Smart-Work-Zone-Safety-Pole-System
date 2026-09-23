"""
Worker Receiver Module: Ingests ESP32-S3 + MPU6500 Worker Wearable Telemetry.
Tracks worker IDs, IMU dynamics, motion states, battery, and BLE/RSSI proximity.
"""

import time
import json
import math
import socket
import select
import threading
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, List
import logging

from ..config import CONFIG

logger = logging.getLogger("WorkerReceiver")

@dataclass
class WorkerState:
    """Represents current state of an active worker wearing an ESP32 tag."""
    worker_id: str = "WORKER_01"
    sequence: int = 0
    ax: float = 0.0          # in g
    ay: float = 0.0
    az: float = 1.0
    gx: float = 0.0          # in deg/s
    gy: float = 0.0
    gz: float = 0.0
    svm: float = 1.0         # Signal Vector Magnitude
    pitch: float = 0.0       # degrees
    roll: float = 0.0        # degrees
    motion_state: str = "STATIC" # STATIC, WALKING, RUNNING, IMPACT, FALL_DETECTED
    battery_v: float = 4.15
    rssi: int = -60          # dBm
    estimated_dist_m: float = 2.0 # Log-distance RSSI estimation
    last_seen: float = field(default_factory=time.time)
    ip_address: Optional[str] = None
    port: Optional[int] = None
    is_active: bool = True

    # Spatial coordinates relative to safety pole (estimated via fusion / RSSI / CV)
    x: float = -3.0          # Inside work zone (negative X)
    y: float = 2.0           # meters from pole along roadway
    vx: float = 0.0          # Lateral velocity m/s
    vy: float = 0.0          # Longitudinal velocity m/s


class WorkerReceiver:
    """
    Asynchronous UDP server managing worker beacons and real-time telemetry.
    Thread-safe registry for all active workers in the work zone.
    """

    def __init__(self, port: Optional[int] = None):
        self.port = port or CONFIG.network.WORKER_UDP_PORT
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._socket: Optional[socket.socket] = None

        # Registry of workers: worker_id -> WorkerState
        self.workers: Dict[str, WorkerState] = {}

    def start(self):
        """Starts worker receiver server thread."""
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._run_server, daemon=True, name="WorkerReceiver")
        self._thread.start()
        logger.info(f"Worker telemetry receiver listening on UDP port {self.port}...")

    def stop(self):
        """Stops worker receiver server."""
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
        logger.info("Worker telemetry receiver stopped.")

    def get_worker(self, worker_id: str) -> Optional[WorkerState]:
        """Retrieves current state for a specific worker ID."""
        with self._lock:
            state = self.workers.get(worker_id)
            if state and (time.time() - state.last_seen < CONFIG.worker.HEARTBEAT_TIMEOUT_SEC):
                return state
            return None

    def get_all_active_workers(self) -> List[WorkerState]:
        """Returns list of currently active worker states."""
        now = time.time()
        active = []
        with self._lock:
            for wid, state in self.workers.items():
                if now - state.last_seen <= CONFIG.worker.HEARTBEAT_TIMEOUT_SEC:
                    state.is_active = True
                    active.append(state)
                else:
                    state.is_active = False
        return active

    def inject_worker_state(self, state: WorkerState):
        """Allows direct injection of simulated worker states for testing."""
        with self._lock:
            state.last_seen = time.time()
            self.workers[state.worker_id] = state

    def send_alert_to_worker(self, worker_id: str, alert_message: str):
        """Sends feedback message over UDP to worker's ESP32 badge (triggers haptic motor)."""
        with self._lock:
            worker = self.workers.get(worker_id)
            if not worker or not worker.ip_address or not worker.port or not self._socket:
                return
            ip = worker.ip_address
            p = worker.port

        try:
            self._socket.sendto(alert_message.encode("utf-8"), (ip, p))
        except Exception as e:
            logger.debug(f"Failed to send alert to worker {worker_id}: {e}")

    def send_evasive_guidance(self, guidance):
        """Transmits actionable evasive directives targeted to the specific worker at risk."""
        if not guidance:
            return

        with self._lock:
            worker = self.workers.get(guidance.target_worker_id)
            if not worker or not worker.ip_address or not worker.port or not self._socket:
                return
            ip = worker.ip_address
            p = worker.port

        try:
            packet = {
                "cmd": "EVASIVE_ACTION",
                "worker_id": guidance.target_worker_id,
                "action": guidance.action_code,
                "directive": guidance.directive_text,
                "urgency": guidance.urgency,
                "haptic": guidance.haptic_pattern,
                "escape_dx": guidance.escape_vector[0],
                "escape_dy": guidance.escape_vector[1],
                "ttc": guidance.ttc_sec
            }
            msg = json.dumps(packet)
            self._socket.sendto(msg.encode("utf-8"), (ip, p))
            logger.info(f"Dispatched evasive guidance to [{guidance.target_worker_id}]: {guidance.directive_text}")
        except Exception as e:
            logger.debug(f"Failed to dispatch evasive guidance: {e}")

    def _estimate_distance_from_rssi(self, rssi: int) -> float:
        """
        Calculates distance using Log-Distance Path Loss Model:
        d = 10 ^ ((A - RSSI) / (10 * n))
        """
        if rssi == 0:
            return 10.0
        A = CONFIG.worker.BLE_TX_POWER_1M
        n = CONFIG.worker.BLE_PATH_LOSS_EXPONENT
        ratio = (A - rssi) / (10.0 * n)
        dist = math.pow(10.0, ratio)
        return max(0.2, min(50.0, dist))

    def _run_server(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._socket.bind((CONFIG.network.HOST, self.port))
            self._socket.setblocking(False)
        except Exception as e:
            logger.warning(f"Could not bind worker UDP port {self.port}: {e}. Mock injection enabled.")
            return

        while self.running:
            try:
                ready = select.select([self._socket], [], [], 0.05)
                if ready[0]:
                    data, addr = self._socket.recvfrom(1024)
                    self._parse_packet(data.decode("utf-8", errors="ignore"), addr)
            except Exception as e:
                if self.running:
                    time.sleep(0.01)

    def _parse_packet(self, raw_str: str, addr: Tuple[str, int]):
        """Parses incoming JSON telemetry packet."""
        try:
            d = json.loads(raw_str)
            wid = str(d.get("worker_id", CONFIG.worker.DEFAULT_WORKER_ID))
            seq = int(d.get("seq", 0))
            ax = float(d.get("ax", 0.0))
            ay = float(d.get("ay", 0.0))
            az = float(d.get("az", 1.0))
            gx = float(d.get("gx", 0.0))
            gy = float(d.get("gy", 0.0))
            gz = float(d.get("gz", 0.0))
            svm = float(d.get("svm", math.sqrt(ax*ax + ay*ay + az*az)))
            pitch = float(d.get("pitch", 0.0))
            roll = float(d.get("roll", 0.0))
            state = str(d.get("state", "STATIC"))
            battery = float(d.get("battery", 4.0))
            rssi = int(d.get("rssi", -60))
            now = time.time()

            dist = self._estimate_distance_from_rssi(rssi)

            with self._lock:
                prev = self.workers.get(wid)
                wx = prev.x if prev else -3.0
                wy = prev.y if prev else 2.0
                vx = prev.vx if prev else 0.0
                vy = prev.vy if prev else 0.0

                self.workers[wid] = WorkerState(
                    worker_id=wid,
                    sequence=seq,
                    ax=ax, ay=ay, az=az,
                    gx=gx, gy=gy, gz=gz,
                    svm=svm,
                    pitch=pitch, roll=roll,
                    motion_state=state,
                    battery_v=battery,
                    rssi=rssi,
                    estimated_dist_m=dist,
                    last_seen=now,
                    ip_address=addr[0],
                    port=addr[1],
                    is_active=True,
                    x=wx, y=wy,
                    vx=vx, vy=vy
                )
        except Exception as e:
            logger.debug(f"Invalid worker packet: {e}")
