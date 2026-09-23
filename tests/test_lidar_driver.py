"""
Unit tests for TSD20 LiDAR driver, range rate estimation, and limits.
"""

import time
import math
from src.sensors.lidar_tsd20 import LidarTSD20, LidarReading
from src.config import CONFIG

def test_lidar_injection_and_range_clamping():
    """Verify LiDAR reading bounds checking."""
    lidar = LidarTSD20(mode="mock")
    lidar.inject_reading(distance_m=12.45, range_rate_mps=-5.2, valid=True)

    reading = lidar.get_latest_reading()
    assert reading.valid is True
    assert math.isclose(reading.distance_m, 12.45, rel_tol=1e-2)
    assert math.isclose(reading.range_rate_mps, -5.2, rel_tol=1e-2)

    # Test out of bounds clamping
    lidar.inject_reading(distance_m=50.0, valid=True) # Exceeds max 20m
    reading = lidar.get_latest_reading()
    assert reading.distance_m <= CONFIG.lidar.MAX_RANGE_M

def test_lidar_json_packet_parsing():
    """Verify parsing of UDP JSON packet from ESP32."""
    lidar = LidarTSD20(mode="wifi", port=5997)
    sample_json = '{"sensor_id":"TSD20_ROAD_01","seq":42,"distance_m":8.35,"range_rate_mps":-8.1,"strength":950,"valid":true}'
    
    lidar._parse_udp_packet(sample_json)
    reading = lidar.get_latest_reading()

    assert reading.valid is True
    assert reading.sensor_id == "TSD20_ROAD_01"
    assert math.isclose(reading.distance_m, 8.35, rel_tol=1e-2)
    assert math.isclose(reading.range_rate_mps, -8.1, rel_tol=1e-2)
    assert reading.signal_strength == 950
