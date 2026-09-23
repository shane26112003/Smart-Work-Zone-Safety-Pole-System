/**
 * Configuration for ESP32 TSD20 LiDAR Wi-Fi Transmitter
 * Smart Work-Zone Safety Pole System
 */

#pragma once

// ==================== IDENTIFICATION ====================
#define LIDAR_SENSOR_ID "TSD20_ROAD_01"

// ==================== WI-FI SETTINGS ====================
#define WIFI_SSID     "SAFETY_POLE_AP"
#define WIFI_PASSWORD "SafetyZone2026!"

// Raspberry Pi 5 Host IP and Dedicated LiDAR UDP Port
#define RPI_IP_ADDRESS "192.168.4.1"
#define LIDAR_UDP_PORT 5006

// ==================== HARDWARE PINOUTS ====================
// ESP32 HardwareSerial 1 or 2 pins connected to TSD20 LiDAR
#define LIDAR_RX_PIN  16   // ESP32 RX connected to TSD20 TX
#define LIDAR_TX_PIN  17   // ESP32 TX connected to TSD20 RX
#define LIDAR_BAUDRATE 115200

#define STATUS_LED_PIN 2

// ==================== SAMPLING ====================
#define LIDAR_SEND_RATE_HZ 50  // 50Hz distance broadcast (20ms interval)
