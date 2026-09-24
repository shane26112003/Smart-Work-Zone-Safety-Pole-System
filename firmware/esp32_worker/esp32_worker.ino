/**
 * Smart Work-Zone Safety Pole System
 * Worker Module Firmware (ESP32-S3 + MPU6500)
 * 
 * Features:
 *  - MPU6500 6-Axis IMU sensor acquisition (Accel + Gyro)
 *  - Onboard Motion Feature Extraction (SVM, Tilt Angles, Variance)
 *  - Multi-state Motion Classifier (STATIC, WALKING, RUNNING, IMPACT, FALL_DETECTED)
 *  - Low-Latency Wi-Fi UDP Telemetry Streaming to Raspberry Pi 5
 *  - BLE Advertisement with Worker ID & Status for RSSI Distance Estimation
 *  - Local Haptic/Audio Feedback for Incoming Vehicle Warnings
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include "config.h"

// ==================== MPU6500 REGISTER MAP ====================
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_ENABLE   0x38
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

// ==================== STATE DEFINITIONS ====================
enum MotionState {
    STATE_STATIC = 0,
    STATE_WALKING = 1,
    STATE_RUNNING = 2,
    STATE_IMPACT = 3,
    STATE_FALL_DETECTED = 4
};

const char* stateNames[] = {"STATIC", "WALKING", "RUNNING", "IMPACT", "FALL_DETECTED"};

// ==================== GLOBALS ====================
WiFiUDP udpClient;
BLEAdvertising *pAdvertising = nullptr;

// Sensor Readings (in physical units: g and deg/s)
float ax = 0.0f, ay = 0.0f, az = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;
float svm = 1.0f;           // Signal Vector Magnitude |a|
float pitch = 0.0f, roll = 0.0f;
MotionState currentMotionState = STATE_STATIC;

// Motion Feature Buffers
const int WINDOW_SIZE = 25;
float svmBuffer[WINDOW_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// Fall Detection State Machine
bool freefallDetected = false;
unsigned long freefallTimestamp = 0;
unsigned long fallLockoutTimestamp = 0;

// Packet Sequence
uint32_t packetSequence = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastImuTime = 0;

// Battery simulation or ADC measurement
float batteryVolts = 4.15f;

// ==================== FORWARD DECLARATIONS ====================
bool initMPU6500();
void readMPU6500();
void updateMotionState();
void broadcastBleBeacon();
void sendUdpTelemetry();
void checkIncomingWarnings();

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n==================================================");
    Serial.printf("[BOOT] Starting ESP32-S3 Worker Tag: %s\n", WORKER_ID);
    Serial.println("==================================================");

    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    // 1. Initialize I2C for MPU6500
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400kHz Fast Mode
    if (!initMPU6500()) {
        Serial.println("[ERROR] MPU6500 Initialization Failed! Check wiring.");
        // Flash LED rapidly to alert user
        while (1) {
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
            delay(100);
        }
    }
    Serial.println("[OK] MPU6500 IMU initialized successfully.");

    // 2. Initialize Wi-Fi Connection
    Serial.printf("[WIFI] Connecting to SSID: %s...\n", WIFI_SSID);
    WiFi.disconnect(true); // Clear any stale radio state
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // Disable modem sleep for ultra-low latency UDP
    if (strlen(WIFI_PASSWORD) > 0) {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    } else {
        WiFi.begin(WIFI_SSID); // Open network (no password)
    }
    
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
        delay(300);
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        Serial.print(".");
        retries++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected! Assigned IP: %s | RSSI: %d dBm\n", 
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        udpClient.begin(UDP_PORT);
    } else {
        Serial.printf("\n[WIFI] Connection Failed (Status code: %d).\n", WiFi.status());
        Serial.println(">>> CHECK: 1) Is Pi 5 Hotspot 2.4GHz? 2) Is SSID/Password correct? <<<");
    }

    // 3. Initialize BLE Beacon
    Serial.println("[BLE] Initializing BLE Beacon...");
    BLEDevice::init(WORKER_ID);
    pAdvertising = BLEDevice::getAdvertising();
    broadcastBleBeacon();
    BLEDevice::startAdvertising();
    Serial.println("[OK] BLE Beacon broadcasting active.");

    digitalWrite(STATUS_LED_PIN, HIGH);
    Serial.println("[READY] Worker Tag ready and operating.");
}

void loop() {
    unsigned long now = millis();

    // 1. IMU Sample Update (50 Hz)
    if (now - lastImuTime >= (1000 / IMU_SAMPLE_RATE_HZ)) {
        lastImuTime = now;
        readMPU6500();
        updateMotionState();
    }

    // 2. Wi-Fi Telemetry Stream (25 Hz)
    if (now - lastTelemetryTime >= (1000 / TELEMETRY_SEND_RATE_HZ)) {
        lastTelemetryTime = now;
        sendUdpTelemetry();
        checkIncomingWarnings();
    }

    // 3. Non-blocking Wi-Fi Reconnect Handling (every 10s)
    static unsigned long lastReconnectAttempt = 0;
    if (WiFi.status() != WL_CONNECTED && (now - lastReconnectAttempt >= 10000)) {
        lastReconnectAttempt = now;
        Serial.println("[WIFI] Reconnecting to AP...");
        WiFi.reconnect();
    }
}

// ==================== MPU6500 HARDWARE FUNCTIONS ====================

bool writeRegister(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU6500_I2C_ADDR);
    Wire.write(reg);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}

uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU6500_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6500_I2C_ADDR, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

bool initMPU6500() {
    uint8_t whoami = readRegister(REG_WHO_AM_I);
    Serial.printf("[MPU6500] WHO_AM_I: 0x%02X (Expected: 0x70 or 0x68)\n", whoami);
    
    // Wake up MPU6500 (clear SLEEP bit)
    writeRegister(REG_PWR_MGMT_1, 0x00);
    delay(10);
    
    // Set Sample Rate Divider: 1kHz / (1 + 9) = 100Hz internal
    writeRegister(REG_SMPLRT_DIV, 0x09);
    
    // Set DLPF (Digital Low Pass Filter) to ~42Hz bandwidth
    writeRegister(REG_CONFIG, 0x03);
    
    // Gyro Full Scale: +/- 2000 deg/s (0x18)
    writeRegister(REG_GYRO_CONFIG, 0x18);
    
    // Accel Full Scale: +/- 8g (0x10) -> Sensitivity: 4096 LSB/g
    writeRegister(REG_ACCEL_CONFIG, 0x10);

    return true;
}

void readMPU6500() {
    Wire.beginTransmission(MPU6500_I2C_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    
    // Request 14 bytes: Accel (6) + Temp (2) + Gyro (6)
    Wire.requestFrom((uint8_t)MPU6500_I2C_ADDR, (uint8_t)14);
    if (Wire.available() >= 14) {
        int16_t rawAx = (Wire.read() << 8) | Wire.read();
        int16_t rawAy = (Wire.read() << 8) | Wire.read();
        int16_t rawAz = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temperature bytes
        int16_t rawGx = (Wire.read() << 8) | Wire.read();
        int16_t rawGy = (Wire.read() << 8) | Wire.read();
        int16_t rawGz = (Wire.read() << 8) | Wire.read();

        // Convert to physical units (+/- 8g scale -> 4096 LSB/g)
        ax = (float)rawAx / 4096.0f;
        ay = (float)rawAy / 4096.0f;
        az = (float)rawAz / 4096.0f;

        // Convert to deg/s (+/- 2000 deg/s -> 16.4 LSB/(deg/s))
        gx = (float)rawGx / 16.4f;
        gy = (float)rawGy / 16.4f;
        gz = (float)rawGz / 16.4f;

        // Signal Vector Magnitude (SVM) in g's
        svm = sqrtf(ax * ax + ay * ay + az * az);

        // Simple roll and pitch estimation (degrees)
        pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
        roll  = atan2f(ay, az) * 57.2957795f;

        // Circular buffer for SVM variance computation
        svmBuffer[bufferIndex] = svm;
        bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
        if (bufferIndex == 0) bufferFull = true;
    }
}

// ==================== MOTION FEATURE & STATE CLASSIFIER ====================

void updateMotionState() {
    unsigned long now = millis();

    // If locked out after a confirmed fall event, hold state for 5 seconds
    if (currentMotionState == STATE_FALL_DETECTED && (now - fallLockoutTimestamp < 5000)) {
        return;
    }

    // 1. Fall Detection State Machine:
    // Phase A: Freefall drop detected (low G condition < 0.35g)
    if (svm < FREEFALL_THRESHOLD_G) {
        freefallDetected = true;
        freefallTimestamp = now;
    }

    // Phase B: Impact detection within 1.0 second of freefall drop (> 3.2g)
    if (freefallDetected && (now - freefallTimestamp <= 1000)) {
        if (svm > IMPACT_THRESHOLD_G) {
            currentMotionState = STATE_FALL_DETECTED;
            fallLockoutTimestamp = now;
            freefallDetected = false;
            Serial.printf("[ALERT] *** WORKER FALL DETECTED! SVM: %.2f g ***\n", svm);
            return;
        }
    } else if (freefallDetected && (now - freefallTimestamp > 1000)) {
        // Freefall expired without high impact
        freefallDetected = false;
    }

    // High impact without preceding freefall (e.g., blunt strike or vehicle tap)
    if (svm > IMPACT_THRESHOLD_G) {
        currentMotionState = STATE_IMPACT;
        return;
    }

    // 2. Normal Locomotion State (Compute variance across SVM buffer)
    int samples = bufferFull ? WINDOW_SIZE : bufferIndex;
    if (samples < 5) return;

    float sum = 0.0f;
    for (int i = 0; i < samples; i++) sum += svmBuffer[i];
    float mean = sum / samples;

    float varianceSum = 0.0f;
    for (int i = 0; i < samples; i++) {
        float diff = svmBuffer[i] - mean;
        varianceSum += diff * diff;
    }
    float variance = varianceSum / samples;

    if (variance < STATIC_VAR_THRESHOLD) {
        currentMotionState = STATE_STATIC;
    } else if (variance < RUNNING_VAR_THRESHOLD) {
        currentMotionState = STATE_WALKING;
    } else {
        currentMotionState = STATE_RUNNING;
    }
}

// ==================== BLE BEACON ADVERTISEMENT ====================

void broadcastBleBeacon() {
    BLEAdvertisementData advData;
    advData.setFlags(0x06); // General Discoverable + BR/EDR not supported
    advData.setName(WORKER_ID);

    // Custom manufacturer payload: 
    // [0..1] Company ID (0xFFFF)
    // [2] Motion State byte
    // [3] Battery percentage
    uint8_t payload[4];
    payload[0] = 0xFF;
    payload[1] = 0xFE;
    payload[2] = (uint8_t)currentMotionState;
    payload[3] = (uint8_t)constrain((int)((batteryVolts - 3.3f) / 0.9f * 100.0f), 0, 100);

    std::string mfgData((char*)payload, sizeof(payload));
    advData.setManufacturerData(mfgData);

    pAdvertising->setAdvertisementData(advData);
}

// ==================== WI-FI UDP TELEMETRY ====================

void sendUdpTelemetry() {
    if (WiFi.status() != WL_CONNECTED) return;

    char jsonBuffer[256];
    snprintf(jsonBuffer, sizeof(jsonBuffer),
        "{\"worker_id\":\"%s\",\"seq\":%u,\"timestamp\":%lu,"
        "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
        "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
        "\"svm\":%.2f,\"pitch\":%.1f,\"roll\":%.1f,"
        "\"state\":\"%s\",\"battery\":%.2f,\"rssi\":%d}",
        WORKER_ID,
        packetSequence++,
        millis(),
        ax, ay, az,
        gx, gy, gz,
        svm, pitch, roll,
        stateNames[currentMotionState],
        batteryVolts,
        WiFi.RSSI()
    );

    udpClient.beginPacket(RPI_IP_ADDRESS, UDP_PORT);
    udpClient.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
    udpClient.endPacket();
}

// ==================== INCOMING EVASIVE GUIDANCE & WARNING LISTENER ====================

void checkIncomingWarnings() {
    int packetSize = udpClient.parsePacket();
    if (packetSize > 0) {
        char buffer[256];
        int len = udpClient.read(buffer, sizeof(buffer) - 1);
        if (len > 0) {
            buffer[len] = '\0';

            // 1. Check for Targeted Evasive Directive JSON packet
            if (strstr(buffer, "EVASIVE_ACTION") != nullptr) {
                Serial.printf("\n[TARGETED ALERT] Directive Received: %s\n", buffer);

                // Rapid evacuation pattern: 3 rapid pulses
                if (strstr(buffer, "CLEAR_ROADWAY") != nullptr || strstr(buffer, "EVACUATE_LEFT") != nullptr) {
                    Serial.println(">>> EVASION DIRECTIVE: [STEP LEFT INTO SAFE ZONE NOW!] <<<");
                    for (int i = 0; i < 3; i++) {
                        digitalWrite(BUZZER_PIN, HIGH);
                        digitalWrite(STATUS_LED_PIN, HIGH);
                        delay(60);
                        digitalWrite(BUZZER_PIN, LOW);
                        digitalWrite(STATUS_LED_PIN, LOW);
                        delay(40);
                    }
                } 
                // Retreat deeper into work zone
                else if (strstr(buffer, "RETREAT") != nullptr) {
                    Serial.println(">>> EVASION DIRECTIVE: [RETREAT DEEPER - VEHICLE BREACHING CONES!] <<<");
                    for (int i = 0; i < 2; i++) {
                        digitalWrite(BUZZER_PIN, HIGH);
                        delay(120);
                        digitalWrite(BUZZER_PIN, LOW);
                        delay(60);
                    }
                }
                // Worker Down / Impact protection
                else if (strstr(buffer, "STAY_DOWN") != nullptr) {
                    Serial.println(">>> EVASION DIRECTIVE: [STAY DOWN & PROTECT HEAD! SIREN ACTIVE] <<<");
                    digitalWrite(BUZZER_PIN, HIGH);
                    digitalWrite(STATUS_LED_PIN, HIGH);
                }
                // Caution warning
                else {
                    digitalWrite(BUZZER_PIN, HIGH);
                    delay(50);
                    digitalWrite(BUZZER_PIN, LOW);
                }
            } 
            // 2. Fallback basic warning strings
            else if (strstr(buffer, "CRITICAL") != nullptr) {
                digitalWrite(BUZZER_PIN, HIGH);
                digitalWrite(STATUS_LED_PIN, HIGH);
            } else if (strstr(buffer, "CLEAR") != nullptr) {
                digitalWrite(BUZZER_PIN, LOW);
                digitalWrite(STATUS_LED_PIN, LOW);
            }
        }
    }
}
