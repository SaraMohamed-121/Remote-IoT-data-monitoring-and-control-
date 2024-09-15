# IoT Remote Data Monitoring and Control System

This Arduino-based project monitors environmental conditions and allows remote control of devices via a web interface using the ESP8266 Wi-Fi module.

## Features:
- **LCD Display**: Displays real-time sensor data (temperature, humidity, smoke).
- **Sensors**:
  - DHT11 (Humidity)
  - MQ-2 (Smoke)
  - LM35 (Temperature)
  - LDR (Light levels)
  - PIR (Motion detection)
- **ESP8266 Wi-Fi**: Provides a web interface to monitor sensor data and control two LEDs remotely.
- **Alarm System**: Triggers when motion or high smoke levels are detected.

### Usage:
- Monitor sensor data and control devices from a web browser over Wi-Fi.

### Components:
- Arduino
- ESP8266
- DHT11, MQ-2, LM35, LDR, PIR
- I2C LCD Display
- LEDs
