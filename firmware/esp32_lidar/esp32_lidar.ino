/**
 * Smart Work-Zone Safety Pole System
 * TSD20 LiDAR Wi-Fi Bridge Firmware (ESP32)
 * 
 * Reads TSD20 ToF LiDAR over UART and transmits low-latency distance,
 * signal quality, and range-rate telemetry over Wi-Fi UDP to Raspberry Pi 5.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

WiFiUDP lidarUdp;
HardwareSerial LidarSerial(2); // UART2

// State variables
float currentDistanceM = 0.0f;
float previousDistanceM = 0.0f;
float rangeRateMps = 0.0f;     // Range rate (negative = approaching vehicle)
uint16_t signalStrength = 0;
uint8_t sensorStatus = 0;
uint32_t packetCounter = 0;
unsigned long lastSendTime = 0;
unsigned long lastSampleTime = 0;

// Parser state for TSD20 / Standard ToF 9-byte packet format:
// Byte 0: 0x59
// Byte 1: 0x59
// Byte 2: Dist_Low
// Byte 3: Dist_High (Dist in cm)
// Byte 4: Strength_Low
// Byte 5: Strength_High
// Byte 6: Temp_Low
// Byte 7: Temp_High
// Byte 8: Checksum (Sum of bytes 0..7 & 0xFF)
uint8_t rxBuffer[9];
int rxIndex = 0;

bool parseLidarByte(uint8_t b) {
    if (rxIndex == 0) {
        if (b == 0x59) {
            rxBuffer[0] = b;
            rxIndex = 1;
        }
        return false;
    } else if (rxIndex == 1) {
        if (b == 0x59) {
            rxBuffer[1] = b;
            rxIndex = 2;
        } else {
            rxIndex = 0;
        }
        return false;
    } else {
        rxBuffer[rxIndex++] = b;
        if (rxIndex == 9) {
            // Verify checksum
            uint16_t check = 0;
            for (int i = 0; i < 8; i++) check += rxBuffer[i];
            if ((check & 0xFF) == rxBuffer[8]) {
                uint16_t distCm = rxBuffer[2] | (rxBuffer[3] << 8);
                signalStrength = rxBuffer[4] | (rxBuffer[5] << 8);
                
                unsigned long now = millis();
                float dt = (now - lastSampleTime) / 1000.0f;
                if (dt > 0.005f && dt < 0.5f) {
                    float newDist = distCm / 100.0f; // meters
                    rangeRateMps = (newDist - previousDistanceM) / dt;
                    previousDistanceM = currentDistanceM;
                    currentDistanceM = newDist;
                } else {
                    currentDistanceM = distCm / 100.0f;
                    previousDistanceM = currentDistanceM;
                }
                lastSampleTime = now;
                rxIndex = 0;
                return true;
            }
            rxIndex = 0; // Checksum failed
        }
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n==================================================");
    Serial.printf("[BOOT] Starting ESP32 TSD20 LiDAR Bridge: %s\n", LIDAR_SENSOR_ID);
    Serial.println("==================================================");

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // Initialize UART connection to TSD20 LiDAR
    LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    Serial.printf("[LIDAR] Listening on UART2 (RX=%d, TX=%d) @ %d baud\n", 
                  LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUDRATE);

    // Connect to Safety Pole Wi-Fi
    Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
    WiFi.disconnect(true); // Clear any stale radio state
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // Ultra-low latency UDP
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
        delay(300);
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected! IP: %s | RSSI: %d dBm\n", 
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        lidarUdp.begin(LIDAR_UDP_PORT);
    } else {
        Serial.printf("\n[WIFI] Connection Failed (Status code: %d).\n", WiFi.status());
        Serial.println(">>> CHECK: 1) Is Pi 5 Hotspot 2.4GHz? 2) Is SSID/Password correct? <<<");
    }

    digitalWrite(STATUS_LED_PIN, HIGH);
    Serial.println("[READY] TSD20 LiDAR streaming operational.");
}

void loop() {
    // 1. Ingest serial stream from TSD20 LiDAR
    while (LidarSerial.available()) {
        uint8_t b = LidarSerial.read();
        parseLidarByte(b);
    }

    // 2. Transmit UDP packet at target rate (50 Hz)
    unsigned long now = millis();
    if (now - lastSendTime >= (1000 / LIDAR_SEND_RATE_HZ)) {
        lastSendTime = now;

        if (WiFi.status() == WL_CONNECTED) {
            char buffer[160];
            snprintf(buffer, sizeof(buffer),
                "{\"sensor_id\":\"%s\",\"seq\":%u,\"timestamp\":%lu,"
                "\"distance_m\":%.3f,\"range_rate_mps\":%.2f,\"strength\":%u,\"valid\":%s}",
                LIDAR_SENSOR_ID,
                packetCounter++,
                now,
                currentDistanceM,
                rangeRateMps,
                signalStrength,
                (currentDistanceM >= 0.10f && currentDistanceM <= 20.0f) ? "true" : "false"
            );

            lidarUdp.beginPacket(RPI_IP_ADDRESS, LIDAR_UDP_PORT);
            lidarUdp.write((const uint8_t*)buffer, strlen(buffer));
            lidarUdp.endPacket();
        }
    }

    // Non-blocking Wi-Fi reconnect check every 10 seconds
    static unsigned long lastReconnectAttempt = 0;
    if (WiFi.status() != WL_CONNECTED && (now - lastReconnectAttempt >= 10000)) {
        lastReconnectAttempt = now;
        Serial.println("[WIFI] Reconnecting...");
        WiFi.reconnect();
    }
}
