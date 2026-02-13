# Distributed Vision-Control System (Phase 1)

This project implements a complete distributed system for real-time face-locked servo control. It consists of a PC-based vision engine, an ESP8266 edge controller, and a cloud-hosted dashboard.

## System Architecture

1.  **Vision Engine (PC)**: Uses a webcam to detect a face, calculate its position, and publish servo angles via MQTT.
2.  **MQTT Broker (VPS)**: Orchestrates communication between the PC, ESP8266, and Dashboard.
3.  **Edge Controller (ESP8266)**: Subscribes to the MQTT broker and actuates a servo motor in real-time.
4.  **Real-Time Dashboard**: A web interface hosted on the VPS to visualize servo positions and system events.

## Components & Setup

### 1. Vision Engine (Python)
- **File**: `src/face_locking.py`
- **Dependencies**: `opencv-python`, `mediapipe`, `paho-mqtt`, `onnxruntime`, `numpy`.
- **Logic**: Tracks a locked face and maps its X-coordinate to a 0-180° angle.
- **Run**: 
  ```bash
  python src/face_locking.py
  ```

### 2. Edge Controller (Arduino/ESP8266)
- **File**: `src/esp8266_servo.ino`
- **Hardware**: ESP8266 (NodeMCU), Servo Motor.
- **Wiring**:
    - **VCC/Red**: `VIN` (5V)
    - **GND/Black**: `GND`
    - **Signal/Yellow**: `D4` (GPIO2)
- **Flash**: Use Arduino IDE with `PubSubClient` and `ArduinoJson` libraries.

### 3. Cloud Backend & Dashboard (VPS)
- **File**: `dashboard/index.html`
- **MQTT Broker**: `157.173.101.159`
- **Persistent Hosting**:
  To keep the dashboard running on the VPS even after you close the terminal, use the following `nohup` command:
  ```bash
  nohup python3 -m http.server 8888 --directory ~/site >> ~/site/http.log 2>&1 &
  ```
  *Note: Make sure your `index.html` is inside the `~/site` directory.*

## Topic Isolation (Team 355)
All communication is isolated to prevent interference with other teams:
- **Servo Control**: `robotics/team355/servo`
- **System Events**: `robotics/team355/events`

## Verification
- **Local**: Check the vision engine console for "Connected to MQTT Broker".
- **Hardware**: The servo should rotate precisely as you move your face.
- **Cloud**: Access the dashboard at `http://157.173.101.159:8888` (if hosted on port 8888).

---
*Created for the Distributed Vision-Control Assignment - Phase 1.*
