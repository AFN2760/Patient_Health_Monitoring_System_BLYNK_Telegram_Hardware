# Patient Health Monitoring System with Blynk, Telegram, and Hardware Alerts

An ESP32-based **proposed prototype** for monitoring patient vital signs and surrounding conditions. The design combines remote viewing in Blynk, Telegram alerts, local LED and buzzer warnings, and automatic ventilation when gas or smoke is detected.

## Components

| Component | Role |
| --- | --- |
| ESP32 | Sensor processing and Wi-Fi connection |
| MAX30102 | Heart rate and SpO₂ |
| AD8232 | ECG signal |
| DS18B20 | Body temperature |
| DHT11 | Ambient temperature and humidity |
| MQ2 | Gas or smoke detection |
| MPU6050 | Fall detection |
| RGB LED and buzzer | Visual and audible alerts |
| Relay or MOSFET, 12 V fan | Ventilation when gas or smoke is detected |

Additional parts: LED resistors, a fan supply, breadboard, and jumper wires.

## How it is intended to work



