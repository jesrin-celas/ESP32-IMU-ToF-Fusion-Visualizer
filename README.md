# 🚀 ESP32 IMU + ToF Fusion Visualizer

Real-time **sensor fusion system** using an ESP32, combining:

- **MPU6050 IMU** → Roll & Pitch estimation  
- **VL53L0X ToF sensor** → Height measurement  
- **Tilt-compensated altitude calculation**  
- **UART telemetry streaming**  
- **Python 3D aircraft visualization**

---

## ✨ Features

✔ Complementary filter for orientation  
✔ Median + exponential filtering for ToF  
✔ Geometry-based height correction  
✔ Real-time pose visualization  
✔ Stable 50 Hz data pipeline  

---

## 🔧 Hardware Requirements

| Component | Purpose |
|----------|---------|
| ESP32 Dev Board | Main controller |
| MPU6050 | Orientation sensing |
| VL53L0X | Distance/height sensing |
| USB Cable | PC communication |
| Jumper Wires | Connections |

---

## 🔌 Wiring Connections (I²C Bus)

| ESP32 Pin | MPU6050 | VL53L0X |
|-----------|---------|---------|
| 3.3V | VCC | VCC |
| GND | GND | GND |
| GPIO 21 | SDA | SDA |
| GPIO 22 | SCL | SCL |

Both sensors share the same **I²C bus**.

---

## 💻 Software Prerequisites

### 🖥 ESP32 Firmware

Install:

- Arduino IDE  
- ESP32 Board Package  
- Adafruit VL53L0X Library  

---

### 🐍 Python Visualizer

```bash
pip install pyserial numpy matplotlib
```
---
## 🚀 How to Run

### 1️⃣ Upload Firmware
- Open `ESP32_Firmware/main_code_uart.ino`
- Select **ESP32 Dev Module**
- Choose the correct COM port
- Upload

### 2️⃣ Run Visualizer
```bash
python Python_Visualizer/imu_tof_visualizer.py
```
---
### 📡 UART Telemetry Format

| Parameter | Value                    |
| --------- | ------------------------ |
| Baud rate | 115200                   |
| Output    | `roll,pitch,height`      |
| Units     | degrees, degrees, meters |
| Rate      | 50 Hz                    |

---
👨‍💻 Author - 
Jesrin Celas A S
