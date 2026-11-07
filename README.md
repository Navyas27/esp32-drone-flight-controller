# ESP32 Autonomous & Glove-Controlled Quadcopter Flight Controller

[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/)
[![Framework: PlatformIO / Arduino](https://img.shields.io/badge/Framework-PlatformIO%20%2F%20Arduino-orange.svg)](https://platformio.org/)
[![Protocol: ESP--NOW](https://img.shields.io/badge/Protocol-ESP--NOW%20Wireless-green.svg)](https://www.espressif.com/en/solutions/low-power-solutions/esp-now)
[![Loop Rate: 500 Hz](https://img.shields.io/badge/Control%20Loop-500%20Hz-brightgreen.svg)]()

A complete, high-performance quadcopter flight controller firmware, sensor calibration suite, and wireless control ecosystem built from the ground up on the dual-core **ESP32**. 

Includes cascaded dual-loop Angle + Rate PID stabilization, MPU6050 6-DOF IMU fusion, HC-SR04 ultrasonic altitude sensing, interactive ESC motor diagnostics, and ultra-low-latency ESP-NOW gesture glove communication.

---

## 📑 Table of Contents
1. [System Architecture](#-system-architecture)
2. [Hardware Specification & Pinout](#-hardware-specification--pinout)
3. [Quad-X Motor Geometry & Mixing](#-quad-x-motor-geometry--mixing)
4. [Control Loops & PID Tuning](#-control-loops--pid-tuning)
5. [Sensor Calibration & Offsets](#-sensor-calibration--offsets)
6. [ESP-NOW Wireless Glove Link](#-esp-now-wireless-glove-link)
7. [Multi-Tier Safety & Failsafes](#-multi-tier-safety--failsafes)
8. [Repository Directory Structure](#-repository-directory-structure)
9. [Build, Flash & Test Instructions](#-build-flash--test-instructions)
10. [Documentation Index](#-documentation-index)

---

## 🏗 System Architecture

The flight controller operates deterministically at **500 Hz (2.0 ms execution loop)** on the ESP32:

```text
 ┌─────────────────────────────────────────────────────────────┐
 │             ESP-NOW Wireless Transmitter Glove             │
 │         (Roll, Pitch, Yaw Rate, Throttle Commands)          │
 └──────────────────────────────┬──────────────────────────────┘
                                │ ESP-NOW Link (2.4 GHz)
                                ▼
 ┌─────────────────────────────────────────────────────────────┐
 │                     ESP32 Flight Controller                 │
 │                                                             │
 │  ┌───────────────────────────────────────────────────────┐  │
 │  │ Outer Angle Loop (P-Controller): Angle -> Target Rate  │  │
 │  └───────────────────────────┬───────────────────────────┘  │
 │                              │ Target Angular Rates (deg/s) │
 │                              ▼                              │
 │  ┌───────────────────────────────────────────────────────┐  │
 │  │ Inner Rate Loop (PID): Target Rate vs MPU6050 Gyro    │  │
 │  └───────────────────────────┬───────────────────────────┘  │
 │                              │ Microsecond Corrections      │
 │                              ▼                              │
 │  ┌───────────────────────────────────────────────────────┐  │
 │  │ Quad-X Motor Mixer & 250 Hz PWM Generator             │  │
 │  └───────────────────────────┬───────────────────────────┘  │
 └──────────────────────────────┼──────────────────────────────┘
                                │
        ┌──────────────┬────────┴─────┬──────────────┐
        ▼              ▼              ▼              ▼
   Front-Right     Rear-Right     Rear-Left      Front-Left
    ESC 1 (CCW)    ESC 2 (CW)    ESC 3 (CCW)    ESC 4 (CW)
```

---

## 🔌 Hardware Specification & Pinout

### ESP32 Pin Assignment Table

| Subsystem | Signal Name | ESP32 GPIO | Electrical Specs | Connected Hardware |
| :--- | :--- | :--- | :--- | :--- |
| **I2C IMU** | `PIN_I2C_SDA` | **GPIO 21** | 3.3V I2C (400 kHz) | MPU6050 `SDA` |
| **I2C IMU** | `PIN_I2C_SCL` | **GPIO 22** | 3.3V I2C (400 kHz) | MPU6050 `SCL` |
| **Motor 1** | `PIN_MOTOR_1` | **GPIO 25** | 250 Hz PWM (1000–2000 µs) | ESC 1 Signal (Front-Right, CCW) |
| **Motor 2** | `PIN_MOTOR_2` | **GPIO 26** | 250 Hz PWM (1000–2000 µs) | ESC 2 Signal (Rear-Right, CW) |
| **Motor 3** | `PIN_MOTOR_3` | **GPIO 27** | 250 Hz PWM (1000–2000 µs) | ESC 3 Signal (Rear-Left, CCW) |
| **Motor 4** | `PIN_MOTOR_4` | **GPIO 14** | 250 Hz PWM (1000–2000 µs) | ESC 4 Signal (Front-Left, CW) |
| **Ultrasonic** | `PIN_TRIG` | **GPIO 18** | 3.3V Digital Output | HC-SR04 `TRIG` |
| **Ultrasonic** | `PIN_ECHO` | **GPIO 19** | 3.3V Safe (via divider) | HC-SR04 `ECHO` (Divided) |
| **Power** | `VIN` | `VIN` | 5V Regulated | 5V 2A–3A BEC Rail |
| **Ground** | `GND` | `GND` | Common Ground Reference | Battery (-) & ESC Signal GND |

---

## 🔄 Quad-X Motor Geometry & Mixing

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

### Mixing Matrix:
$$\begin{aligned}
M_1\ (\text{FR, CCW}) &= \text{Throttle} - \text{Pitch} - \text{Roll} + \text{Yaw} \\
M_2\ (\text{RR, CW})  &= \text{Throttle} + \text{Pitch} - \text{Roll} - \text{Yaw} \\
M_3\ (\text{RL, CCW}) &= \text{Throttle} + \text{Pitch} + \text{Roll} + \text{Yaw} \\
M_4\ (\text{FL, CW})  &= \text{Throttle} - \text{Pitch} + \text{Roll} - \text{Yaw}
\end{aligned}$$

---

## 🎛 Control Loops & PID Tuning

### Default PID Gains

```cpp
// Outer Loop: Angle P-Controller
const float ANGLE_KP_ROLL     = 4.0f;
const float ANGLE_KP_PITCH    = 4.0f;

// Inner Loop: Rate PID Controllers
// Roll Rate PID
const float RATE_KP_ROLL      = 0.65f;
const float RATE_KI_ROLL      = 0.25f;
const float RATE_KD_ROLL      = 0.020f;

// Pitch Rate PID
const float RATE_KP_PITCH     = 0.65f;
const float RATE_KI_PITCH     = 0.25f;
const float RATE_KD_PITCH     = 0.020f;

// Yaw Rate PID (Direct rate loop)
const float RATE_KP_YAW       = 2.00f;
const float RATE_KI_YAW       = 0.40f;
const float RATE_KD_YAW       = 0.000f;

// Clamps & Anti-Windup (microseconds)
const float PID_I_LIMIT_ROLL  = 120.0f;
const float PID_I_LIMIT_PITCH = 120.0f;
const float PID_I_LIMIT_YAW   = 100.0f;
const float PID_OUT_LIMIT     = 350.0f;
```

---

## 🎯 Sensor Calibration & Offsets

### MPU6050 IMU Calibrated Constants
Generated via stationary 2,000-sample empirical calibration:
```cpp
const float ACCEL_X_OFFSET = -863.19f;
const float ACCEL_Y_OFFSET = -19.86f;
const float ACCEL_Z_OFFSET = 12188.75f;

const float GYRO_X_OFFSET  = -53.10f;
const float GYRO_Y_OFFSET  = 97.94f;
const float GYRO_Z_OFFSET  = 20.92f;
```

### Ultrasonic Altitude Rangefinder
- **Echo Timeout**: 25 ms (~4.2 m range limit)
- **Temperature Compensation**: Automatic speed-of-sound adjustment using MPU6050 die temperature:
  $$v_{\text{sound}} = 331.3 + (0.606 \times T_{^\circ\text{C}})\quad [\text{m/s}]$$
- **Median Filter**: 25-sample median window rejecting prop acoustic wash.
- **Ground Offset**: Calibrated post-mounting to set skid contact height as `0.0 cm`.

---

## 📡 ESP-NOW Wireless Glove Link

- **Drone MAC Address (STA)**: `78:1C:3C:CB:CF:A8`
- **Wi-Fi Channel**: Channel 1 (2.4 GHz)
- **Command Packet Struct**:
```cpp
typedef struct __attribute__((packed)) {
  int16_t roll;      // Roll tilt:    -500 .. +500 -> mapped to -30° .. +30°
  int16_t pitch;     // Pitch tilt:   -500 .. +500 -> mapped to -30° .. +30°
  int16_t yaw_rate;  // Yaw rate:     -500 .. +500 -> mapped to -150°/s .. +150°/s
  int16_t throttle;  // Throttle PWM: 1000 .. 2000 µs
} GloveCommand;
```

---

## 🛡 Multi-Tier Safety & Failsafes

1. **Boot Disarm**: All 4 ESC channels lock to $1000\,\mu\text{s}$ immediately on boot.
2. **2-Second Low-Throttle Arming Interlock**: Throttle must remain below $1050\,\mu\text{s}$ for 2000 ms before arming activates.
3. **500 ms Packet Timeout Failsafe**: Automatic motor cut if wireless communication is lost for $> 500\,\text{ms}$.
4. **Emergency Serial Kill Switch**: Instant motor shutdown upon sending `k` or `K` over Serial (115200 baud).
5. **Integral Anti-Windup & Ground Reset**: I-terms zero out when throttle is idle, eliminating aggressive motor surge at takeoff.

---

## 📁 Repository Directory Structure

```text
├── docs/                                  # Complete Technical Documentation
│   ├── flight_controller_doc.md          # Control architecture & loop math
│   ├── hardware_and_wiring.md            # Wiring diagrams, BOM & pinout
│   ├── mpu6050_calibration_guide.md      # IMU offsets & calibration math
│   ├── esc_testing_guide.md              # ESC calibration & interactive CLI
│   ├── ultrasonic_rangefinder_guide.md   # HC-SR04 altitude hold & safety divider
│   ├── glove_controller_spec.md          # ESP-NOW protocol & glove gestures
│   └── sensor_telemetry_report.md        # Bench validation & sensor data
│
├── flight_controller/                     # Primary Arduino Firmware
│   └── flight_controller.ino             # Cascaded 500 Hz Flight Controller
│
├── src/                                   # PlatformIO Source Mirror
│   └── main.cpp                          # C++ Flight Controller source
│
├── include/                               # C/C++ Headers
│
├── esc_test/                              # ESC Diagnostics & Motor CLI
│   └── esc_test.ino                      # Interactive motor test utility
│
├── mpu_calibrate/                         # IMU Calibration Utility
│   └── mpu_calibrate.ino                 # 2,000-sample offset generator
│
├── ultrasonic_calibrate/                  # Rangefinder Calibration Utility
│   └── ultrasonic_calibrate.ino          # Median filter & temp compensation
│
├── sensor_calibration/                    # ADXL345 / Testbench Calibration
│   └── sensor_calibration.ino            # Live accelerometer telemetry
│
├── mpu6050_test/                          # Arduino Uno IMU Bringup Sketch
│   └── mpu6050_test.ino
│
├── mpu_esp32_test/                        # ESP32 IMU Diagnostic Sketch
│   └── mpu_esp32_test.ino
│
├── platformio.ini                         # PlatformIO Project Configuration
└── README.md                              # Main Project Documentation
```

---

## 🚀 Build, Flash & Test Instructions

### Using PlatformIO (VS Code or CLI)
```bash
# Build firmware
pio run

# Upload to ESP32
pio run --target upload

# Open Serial Monitor
pio device monitor -b 115200
```

### Using Arduino CLI
```powershell
# Compile & Upload Flight Controller
arduino-cli compile --fqbn esp32:esp32:esp32 "d:\drone\flight_controller"
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32 "d:\drone\flight_controller"

# Open Monitor
arduino-cli monitor -p COM3 --config baudrate=115200
```

---

## 📖 Documentation Index
- [Detailed Flight Controller Architecture](docs/flight_controller_doc.md)
- [Complete Hardware & Wiring Guide](docs/hardware_and_wiring.md)
- [MPU6050 Calibration Manual](docs/mpu6050_calibration_guide.md)
- [ESC Testing & Bringup Guide](docs/esc_testing_guide.md)
- [Ultrasonic Rangefinder Guide](docs/ultrasonic_rangefinder_guide.md)
- [Wireless Glove Controller Spec](docs/glove_controller_spec.md)
- [Sensor Bench Telemetry Report](docs/sensor_telemetry_report.md)
