# Smart Work-Zone Safety Pole System - Hardware Wiring & Pinouts

## 1. Central Unit: Raspberry Pi 5

The Raspberry Pi 5 operates as the central edge AI and sensor fusion processor.

```
                         Raspberry Pi 5 40-Pin Header
                                3.3V  [ 1] [ 2]  5.0V (Relay VCC)
              I2C SDA (GPIO 2)  [ 3] [ 4]  5.0V
              I2C SCL (GPIO 3)  [ 5] [ 6]  GND
                                [ 7] [ 8]  UART TX (GPIO 14) (Optional direct LiDAR)
                           GND  [ 9] [10]  UART RX (GPIO 15) (Optional direct LiDAR)
                                [11] [12]  
           DANGER RED LED (17)  [11] [12]  
          CAUTION AMB LED (22)  [15] [16]  
                                [17] [18]  
       SIREN 12V RELAY IN (27)  [13] [14]  GND
             AUDIO BUZZER (23)  [16] [17]  
```

### Pi 5 Peripheral Connections:
1. **Road-Facing Camera (Raspberry Pi Camera Module Rev 1.3 - OV5647 5MP)**:
   - **Crucial Pi 5 Ribbon Cable Note**:
     - The Raspberry Pi 5 uses smaller **22-pin 0.5mm pitch** MIPI CSI connectors (`CAM/DISP 0` and `CAM/DISP 1`).
     - The Camera Module Rev 1.3 board uses a **15-pin 1.0mm pitch** socket.
     - You **must use a 15-pin to 22-pin adapter ribbon cable** (standard Raspberry Pi 5 / Zero camera cable). The shiny contacts face towards the HDMI ports on the Pi 5.
   - **Device Tree Overlay (/boot/firmware/config.txt)**:
     - On Raspberry Pi OS Bookworm, add or ensure the following line is present:
       ```ini
       dtoverlay=ov5647
       ```
   - **Quick Hardware Test Command**:
     ```bash
     rpicam-hello -t 3000
     ```
2. **Work-Zone Camera**: Connects to USB 3.0 port (HD Webcam).
3. **Wi-Fi Interface**: Onboard 802.11ac Wi-Fi set to Access Point (AP) mode (`SSID: SAFETY_POLE_AP`).
4. **Warning Actuator Outputs**:
   - `GPIO 17`: Red High-Intensity Strobe flasher (via 2N2222 NPN or MOSFET driver).
   - `GPIO 22`: Amber Warning Strobe flasher.
   - `GPIO 27`: 12V Siren Relay IN (Active LOW with optocoupler isolation).
   - `GPIO 23`: 5V Active Piezo Buzzer.

---

## 2. Worker Tag: ESP32-S3 + MPU6500

Each worker carries a wearable safety badge powered by a rechargeable 3.7V LiPo battery.

```
       ESP32-S3                        MPU6500 IMU
     ┌───────────┐                    ┌───────────┐
     │      3.3V ├────────────────────┤ VCC       │
     │       GND ├────────────────────┤ GND       │
     │    GPIO 8 ├──────[SDA]─────────┤ SDA       │
     │    GPIO 9 ├──────[SCL]─────────┤ SCL       │
     │       GND ├────────────────────┤ AD0 (0x68)│
     │   GPIO 10 ├──────[INT]─────────┤ INT       │
     │           │                    └───────────┘
     │    GPIO 2 ├──[330Ω]──► Status LED
     │    GPIO 1 ├──► Haptic Motor / Local Alert Buzzer
     │           │
     │  LiPo Bat ├──► TP4056 USB-C Charger & Protection
     └───────────┘
```

---

## 3. Road LiDAR: ESP32 + TSD20 ToF LiDAR

Mounted on the road-facing arm of the Safety Pole, aimed at approaching traffic.

```
       ESP32 Node                      TSD20 LiDAR (3.3V)
     ┌───────────┐                    ┌───────────┐
     │      3.3V ├────────────────────┤ VCC (3.3V)│
     │       GND ├────────────────────┤ GND       │
     │   GPIO 16 ├──────[RX2]◄────────┤ TX        │
     │   GPIO 17 ├──────[TX2]────────►┤ RX        │
     └───────────┘                    └───────────┘
```

*Note: With 3.3V power, UART logic levels match the ESP32 natively (no level-shifter needed). The ESP32 streams the parsed distance frames at 50Hz over Wi-Fi UDP to port 5006 on the Raspberry Pi 5.*
