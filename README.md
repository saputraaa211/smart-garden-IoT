## 🌱 Smart Garden IoT System

Smart Garden IoT System is an IoT-based automatic plant watering system built around the ESP32 microcontroller. The system was designed to monitor environmental conditions and make adaptive irrigation decisions using the Mamdani Fuzzy Logic method.

This project was originally developed as a complete end-to-end IoT architecture consisting of:

- ESP32 firmware (sensor acquisition & pump control)
- FastAPI backend server
- Real-time web monitoring client
- WebSocket-based real-time communication
At present, this repository contains the ESP32 firmware, which represents the core logic and control layer of the system.

## 🎯 System Overview

The system collects environmental data from:

- Soil moisture sensor
- Soil temperature sensor
- Air humidity sensor (from DHT 11)
- Air temperature sensor (from DHT 11)

Instead of using a simple threshold-based approach, irrigation decisions are determined through a Fuzzy Logic inference system. This allows the system to evaluate multiple environmental parameters simultaneously and produce adaptive watering behavior.

The ESP32 is responsible for:
- Sensor data acquisitio
- Fuzzy inference processing
- Water pump control
- Data transmission to backend server
  
## 🏗 Original Architecture
```bash 
Sensors → ESP32 (Fuzzy Logic Engine) → FastAPI Backend (WebSocket Server) → Web Monitoring Client (Real-Time UI)```

The system utilized WebSocket communication to enable real-time, bidirectional data exchange between the backend server and the web client. This approach allowed instant status updates and responsive control without polling delays.

## 📦 Repository Status

This repository currently includes:

✅ ESP32 firmware source code

❌ FastAPI backend (not included)

❌ Web monitoring client (not included)

The backend and web client were developed as part of the original architecture but are not available in this repository version.

This documentation is provided to transparently describe the intended full system design.

## 🛠 Firmware Technologies

- ESP32
- Mamdani Fuzzy Logic implementation
- Soil moisture sensor integration
- Soil temperatur sensor
- DHT11 sensor integration
- WebSocket-based communication architecture
  
## 🚀Engineering Highlights

This project demonstrates:

- End-to-end IoT system architecture design
- Embedded implementation of Fuzzy Logic
- Real-time data from sensor
- WebSocket-based real-time communication design
- Adaptive irrigation control beyond simple threshold systems
The backend and web monitoring components can be redeveloped based on the documented architecture if required.
