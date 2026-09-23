"""
TSD20 LiDAR Driver and Ingestion Module.
Supports:
 1. Wi-Fi UDP stream from ESP32 bridge node (Default mode)
 2. Direct UART / Serial interface (RPi 5 GPIO /dev/ttyAMA0 or USB-UART /dev/ttyUSB0)
 3. High-fidelity Mock/Simulation mode for testing without hardware
"""

import time
import json
import socket
import select
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Deque, Tuple
import logging

from ..config import CONFIG

logger = logging.getLogger("LidarTSD20")

@dataclass
class LidarReading:
    """Represents a single measurement sample from TSD20 LiDAR."""
    distance_m: float = 20.0
    range_rate_mps: float = 0.0     # Negative = approaching, Positive = receding
    signal_strength: int = 100
    valid: bool = False
    timestamp: float = field(default_factory=time.time)
    sensor_id: str = "TSD20_ROAD_01"


class LidarTSD20:
    """
    Manages TSD20 LiDAR acquisition over Wi-Fi UDP or Serial UART.
    Thread-safe, non-blocking access to latest range measurements.
    """

    def __init__(self, mode: Optional[str] = None, port: Optional[int] = None, serial_port: Optional[str] = None):
        self.mode = mode or CONFIG.lidar.SOURCE_TYPE
        self.udp_port = port or CONFIG.network.LIDAR_UDP_PORT
        self.serial_port = serial_port or CONFIG.lidar.SERIAL_PORT
        self.baud_rate = CONFIG.lidar.BAUD_RATE
        
        self.running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Latest state
        self._latest_reading = LidarReading()
        self._history: Deque[Tuple[float, float]] = deque(maxlen=20) # (time, distance)

        # Mock generator variables
        self._mock_dist = 20.0
        self._mock_rate = -5.0 # m/s

    def start(self):
        """Starts background reader thread."""
        if self.running:
            return
        self.running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True, name="LidarReader")
        self._thread.start()
        logger.info(f"TSD20 LiDAR driver started in [{self.mode}] mode.")

    def stop(self):
        """Stops background reader thread."""
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logger.info("TSD20 LiDAR driver stopped.")

    def get_latest_reading(self) -> LidarReading:
        """Returns the most recent thread-safe LiDAR measurement."""
        with self._lock:
            # Copy to avoid race conditions
            reading = LidarReading(
                distance_m=self._latest_reading.distance_m,
                range_rate_mps=self._latest_reading.range_rate_mps,
                signal_strength=self._latest_reading.signal_strength,
                valid=self._latest_reading.valid,
                timestamp=self._latest_reading.timestamp,
                sensor_id=self._latest_reading.sensor_id
            )
            return reading

    def inject_reading(self, distance_m: float, range_rate_mps: float = 0.0, valid: bool = True):
        """Allows direct injection of simulated or mock readings."""
        with self._lock:
            now = time.time()
            self._history.append((now, distance_m))
            self._latest_reading = LidarReading(
                distance_m=max(CONFIG.lidar.MIN_RANGE_M, min(CONFIG.lidar.MAX_RANGE_M, distance_m)),
                range_rate_mps=range_rate_mps,
                signal_strength=1000 if valid else 0,
                valid=valid,
                timestamp=now,
                sensor_id="MOCK_LIDAR"
            )

    def _run_loop(self):
        if self.mode == "wifi":
            self._run_udp_server()
        elif self.mode == "serial":
            self._run_serial_reader()
        else:
            self._run_mock_generator()

    def _run_udp_server(self):
        """Listens on UDP socket for JSON frames from ESP32."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((CONFIG.network.HOST, self.udp_port))
            sock.setblocking(False)
            logger.info(f"LiDAR UDP server listening on port {self.udp_port}...")
        except Exception as e:
            logger.error(f"Failed to bind LiDAR UDP port {self.udp_port}: {e}. Falling back to mock mode.")
            self._run_mock_generator()
            return

        while self.running:
            try:
                ready = select.select([sock], [], [], 0.05)
                if ready[0]:
                    data, addr = sock.recvfrom(512)
                    self._parse_udp_packet(data.decode("utf-8", errors="ignore"))
            except Exception as e:
                if self.running:
                    time.sleep(0.01)

        sock.close()

    def _parse_udp_packet(self, payload_str: str):
        """Parses incoming JSON from ESP32."""
        try:
            data = json.loads(payload_str)
            dist = float(data.get("distance_m", 20.0))
            rate = float(data.get("range_rate_mps", 0.0))
            strength = int(data.get("strength", 100))
            valid = bool(data.get("valid", True))
            sensor_id = str(data.get("sensor_id", "TSD20_ROAD_01"))

            now = time.time()
            with self._lock:
                self._history.append((now, dist))
                
                # Filter range rate if not computed on ESP32
                if rate == 0.0 and len(self._history) >= 2:
                    dt = self._history[-1][0] - self._history[-2][0]
                    if dt > 0.005:
                        rate = (self._history[-1][1] - self._history[-2][1]) / dt

                self._latest_reading = LidarReading(
                    distance_m=dist,
                    range_rate_mps=rate,
                    signal_strength=strength,
                    valid=valid and (CONFIG.lidar.MIN_RANGE_M <= dist <= CONFIG.lidar.MAX_RANGE_M),
                    timestamp=now,
                    sensor_id=sensor_id
                )
        except Exception as err:
            logger.debug(f"Failed to parse LiDAR JSON packet: {err}")

    def _run_serial_reader(self):
        """Reads directly from UART serial port (if hardware connected)."""
        try:
            import serial
            ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            logger.info(f"Opened Serial port {self.serial_port} for TSD20.")
        except Exception as e:
            logger.warning(f"Serial port {self.serial_port} unavailable ({e}). Falling back to mock mode.")
            self._run_mock_generator()
            return

        rx_buf = bytearray()
        while self.running:
            try:
                raw = ser.read(32)
                if raw:
                    rx_buf.extend(raw)
                    # Search for standard 9-byte ToF header 0x59 0x59
                    while len(rx_buf) >= 9:
                        if rx_buf[0] == 0x59 and rx_buf[1] == 0x59:
                            pkt = rx_buf[:9]
                            rx_buf = rx_buf[9:]
                            
                            # Checksum verify
                            chk = sum(pkt[:8]) & 0xFF
                            if chk == pkt[8]:
                                dist_cm = pkt[2] | (pkt[3] << 8)
                                strength = pkt[4] | (pkt[5] << 8)
                                dist_m = dist_cm / 100.0
                                
                                now = time.time()
                                with self._lock:
                                    prev_t, prev_d = self._history[-1] if self._history else (now, dist_m)
                                    dt = now - prev_t
                                    rate = (dist_m - prev_d) / dt if dt > 0.005 else 0.0
                                    self._history.append((now, dist_m))

                                    self._latest_reading = LidarReading(
                                        distance_m=dist_m,
                                        range_rate_mps=rate,
                                        signal_strength=strength,
                                        valid=(CONFIG.lidar.MIN_RANGE_M <= dist_m <= CONFIG.lidar.MAX_RANGE_M),
                                        timestamp=now,
                                        sensor_id="TSD20_SERIAL"
                                    )
                        else:
                            rx_buf.pop(0)
                else:
                    time.sleep(0.01)
            except Exception as e:
                time.sleep(0.02)

        ser.close()

    def _run_mock_generator(self):
        """Generates continuous simulated range readings for approaching / passing traffic."""
        logger.info("TSD20 running in Mock Generator mode.")
        while self.running:
            # Simulate a car approaching from 20m down to 2m, then resetting
            self._mock_dist += self._mock_rate * 0.02
            if self._mock_dist <= 2.0:
                self._mock_dist = 20.0
                self._mock_rate = -4.0 - (time.time() % 3.0) # between -4 and -7 m/s (~15-25 km/h)
            
            with self._lock:
                now = time.time()
                self._latest_reading = LidarReading(
                    distance_m=round(self._mock_dist, 3),
                    range_rate_mps=round(self._mock_rate, 2),
                    signal_strength=850,
                    valid=True,
                    timestamp=now,
                    sensor_id="TSD20_MOCK"
                )
            time.sleep(0.02) # 50 Hz
