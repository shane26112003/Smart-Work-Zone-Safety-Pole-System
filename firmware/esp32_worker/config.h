/**
 * Configuration for ESP32-S3 Worker Safety Tag
 * Smart Work-Zone Safety Pole System
 */

#pragma once

// ==================== WORKER IDENTIFICATION ====================
#define WORKER_ID "WORKER_01"

// ==================== WI-FI SETTINGS ====================
// Replace with your Work-Zone Safety Pole Access Point or Hotspot
#define WIFI_SSID     "SAFETY_POLE_AP"
#define WIFI_PASSWORD "SafetyZone2026!"

// Raspberry Pi 5 IP Address and UDP Port
#define RPI_IP_ADDRESS "10.66.40.6"  // Phone hotspot IP for Pi 5
#define UDP_PORT       5005

// ==================== HARDWARE PINOUTS ====================
// ESP32-S3 Default I2C Pins for MPU6500
#define I2C_SDA_PIN    8
#define I2C_SCL_PIN    9
#define STATUS_LED_PIN 2     // Onboard LED or external warning indicator
#define BUZZER_PIN     1     // Small haptic vibrator / buzzer for local worker alerts

// ==================== MPU6500 I2C ADDRESS ====================
#define MPU6500_I2C_ADDR 0x68 // AD0 connected to GND (0x69 if connected to VCC)

// ==================== SAMPLING & TIMING ====================
#define IMU_SAMPLE_RATE_HZ     50    // 50Hz IMU read rate (20ms interval)
#define TELEMETRY_SEND_RATE_HZ 25    // 25Hz transmission rate (40ms interval)

// ==================== MOTION & FALL DETECTION THRESHOLDS ====================
#define FREEFALL_THRESHOLD_G  0.35f  // Low G-force indicating freefall
#define IMPACT_THRESHOLD_G    3.20f  // High G-force spike indicating impact
#define STATIC_VAR_THRESHOLD  0.04f  // Low variance for stationary worker
#define RUNNING_VAR_THRESHOLD 0.55f  // High variance for running worker
