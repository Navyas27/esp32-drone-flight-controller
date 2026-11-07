# Drone Hardware & Wiring Specification

This document details the complete hardware configuration, pin assignments, electrical wiring, power distribution, and safety guidelines for the ESP32-based Quadcopter.

---

## 1. System Overview & Bill of Materials (BOM)

| Component | Specification / Part | Purpose | Working Voltage |
| :--- | :--- | :--- | :--- |
| **Flight Controller MCU** | ESP32-WROOM-32 DevKit (30/38 pin) | Main processing unit (500 Hz control loop, ESP-NOW) | 3.3V logic (5V via USB/BEC) |
| **Primary IMU** | InvenSense MPU6050 (GY-521) | 6-DOF Accelerometer + Gyroscope | 3.3V (VCC) / I2C Bus |
| **Secondary Accelerometer** | Analog Devices ADXL345 | 3-axis accelerometer testbench identification | 3.3V / I2C Bus |
| **Altitude / Proximity Sensor** | HC-SR04 / US-100 / RCWL-1601 | Ultrasonic rangefinder for ground detection & altitude | 5V / 3.3V (Divider required on Echo for 5V) |
| **Electronic Speed Controllers**| 4x 20A–30A BLDC ESCs | Brushless motor drivers | LiPo Battery (2S–4S) |
| **Motors** | 4x Brushless DC Motors (e.g. 2212 / 920-1000KV) | Quadcopter propulsion (Quad-X setup) | Direct from ESC |
| **Propellers** | 2x CW + 2x CCW (e.g. 1045 or 8045) | Thrust generation | - |
| **Power Distribution** | PDB / XT60 Harness + 5V BEC | Powers ESCs directly & steps down to 5V for ESP32 | Battery Voltage -> 5V Regulated |
| **Remote Controller** | ESP32 Hand Gesture / Transmitter Glove | Wireless command transmitter via ESP-NOW | 3.3V / Battery |

---

## 2. Complete ESP32 GPIO Pin Assignment

| Subsystem | Signal / Function | ESP32 GPIO | Electrical Characteristics | Target Peripheral Pin |
| :--- | :--- | :--- | :--- | :--- |
| **I2C Bus** | `PIN_I2C_SDA` | **GPIO 21** | 3.3V I2C Data (400 kHz Fast Mode) | MPU6050 `SDA` |
| **I2C Bus** | `PIN_I2C_SCL` | **GPIO 22** | 3.3V I2C Clock (400 kHz Fast Mode) | MPU6050 `SCL` |
| **Motor 1** | `PIN_MOTOR_1` | **GPIO 25** | 250 Hz ESC PWM (1000–2000 µs) | ESC 1 Signal (Front-Right, CCW) |
| **Motor 2** | `PIN_MOTOR_2` | **GPIO 26** | 250 Hz ESC PWM (1000–2000 µs) | ESC 2 Signal (Rear-Right, CW) |
| **Motor 3** | `PIN_MOTOR_3` | **GPIO 27** | 250 Hz ESC PWM (1000–2000 µs) | ESC 3 Signal (Rear-Left, CCW) |
| **Motor 4** | `PIN_MOTOR_4` | **GPIO 14** | 250 Hz ESC PWM (1000–2000 µs) | ESC 4 Signal (Front-Left, CW) |
| **Ultrasonic** | `PIN_TRIG` | **GPIO 18** | 3.3V Digital Output (10 µs pulse) | HC-SR04 `TRIG` |
| **Ultrasonic** | `PIN_ECHO` | **GPIO 19** | 3.3V Safe Input (via divider) | HC-SR04 `ECHO` (divided) |
| **Power** | `VIN / 5V` | `VIN` | 5V DC from BEC / Step-Down | PDB 5V Regulated Rail |
| **Ground** | `GND` | `GND` | Common Ground Reference | Battery (-) & All ESC Signal GNDs |

---

## 3. Quad-X Motor Geometry & Rotation

```text
                  FRONT
                    ▲
                    │
         M4 (FL)         M1 (FR)
          [CW]            [CCW]
             \            /
              \    ▲     /
               \   │    /
                   │
               /   │    \
              /          \
             /            \
          [CCW]          [CW]
         M3 (RL)         M2 (RR)
                    │
                  REAR
```

### Motor Rotation & Propeller Rules:
1. **Motor 1 (Front-Right)**: Counter-Clockwise (**CCW**) rotation. Requires CCW propeller.
2. **Motor 2 (Rear-Right)**: Clockwise (**CW**) rotation. Requires CW propeller.
3. **Motor 3 (Rear-Left)**: Counter-Clockwise (**CCW**) rotation. Requires CCW propeller.
4. **Motor 4 (Front-Left)**: Clockwise (**CW**) rotation. Requires CW propeller.

---

## 4. Ultrasonic Sensor (HC-SR04) Voltage Divider Safety

> [!WARNING]
> Standard HC-SR04 modules output a **5V logic pulse** on their ECHO pin. Connecting 5V directly to an ESP32 GPIO will damage the input pin over time.

### Voltage Divider Circuit:
```text
HC-SR04 Echo (5V) ────[ R1: 1kΩ - 2.2kΩ ]────┬──── ESP32 GPIO 19 (3.3V Safe)
                                             │
                                     [ R2: 2kΩ - 3.3kΩ ]
                                             │
                                            GND
```
Formula:
$$V_{\text{out}} = 5.0\,\text{V} \times \frac{R_2}{R_1 + R_2} \approx 3.1\,\text{V} \sim 3.3\,\text{V}$$

*Alternative: If using 3.3V native sensors such as HC-SR04P, US-100, or RCWL-1601, you can connect directly without a divider.*

---

## 5. Power & Grounding Rules

1. **Common Ground is Mandatory**: The ESP32 `GND` must be firmly connected to the LiPo battery negative terminal and all 4 ESC ground leads. Without a unified ground, PWM reference voltages float, causing motor jitter or signal loss.
2. **Do Not Power Motors from ESP32**: Motors and ESCs must draw main current directly from the LiPo battery / PDB.
3. **Clean Logic Power**: Power the ESP32 `VIN` pin using a dedicated 5V 2A–3A BEC (Battery Eliminator Circuit) or DC-DC step-down buck converter with decoupling capacitors.
