# Smart Work-Zone Safety Pole System - Bill of Materials (BOM)

| Item | Component | Specification | Quantity | Purpose |
| :--- | :--- | :--- | :---: | :--- |
| 1 | **Raspberry Pi 5** | 4GB or 8GB RAM, Quad-Core ARM Cortex-A76 @ 2.4GHz | 1 | Central edge processing & decision unit |
| 2 | **RPi Camera Module Rev 1.3** | 5MP OmniVision OV5647 + 22-pin to 15-pin RPi 5 adapter ribbon | 1 | Road-facing vehicle and hazard detection |
| 3 | **USB Webcam** | 1080p / 720p HD USB 3.0 wide field of view | 1 | Work-zone area worker monitoring |
| 4 | **TSD20 ToF LiDAR** | 0.1m - 20m range, 50-100Hz sampling, UART / Serial | 1 | Road vehicle distance & range rate measurement |
| 5 | **ESP32-S3 Dev Board** | Dual-core Xtensa LX7 @ 240MHz, Wi-Fi & BLE 5.0 | 1 | Worker wearable module processor |
| 6 | **ESP32 Dev Board** | Standard ESP32 (WROOM-32) | 1 | TSD20 LiDAR Wi-Fi transmitter bridge |
| 7 | **MPU6500 6-Axis IMU** | 3-axis Accelerometer ($\pm 8g$) + 3-axis Gyroscope ($\pm 2000^\circ/\text{s}$), I2C | 1 | Worker motion & fall/impact sensor |
| 8 | **LiPo Battery + Charger** | 3.7V 1200mAh LiPo + TP4056 USB-C charging module | 1 | Worker tag power supply |
| 9 | **Directional Warning LEDs** | 12V High-Intensity Amber & Red Flasher Modules | 2 | Visual warnings towards oncoming vehicles |
| 10 | **Smart Siren / Horn** | 12V 110dB Siren horn | 1 | Audible high-priority collision warning |
| 11 | **Relay Module** | 5V 1-Channel Relay with optocoupler isolation | 1 | Switching 12V siren from Pi 5 GPIO |
| 12 | **Active Piezo Buzzer** | 5V continuous audio buzzer | 1 | Caution chirp alert |
| 13 | **Step-Down Converter** | 12V to 5V 5A DC-DC Buck converter | 1 | Powering Pi 5 and sensors from 12V battery |
| 14 | **Safety Pole Enclosure** | IP66 Weatherproof outdoor enclosure & mount pole | 1 | Field deployment housing |
