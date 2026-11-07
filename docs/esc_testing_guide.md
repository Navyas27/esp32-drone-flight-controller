# Electronic Speed Controller (ESC) Testing & Calibration Guide

## 1. Safety Rules

> [!CAUTION]
> **ALWAYS REMOVE PROPELLERS BEFORE BENCH TESTING MOTORS.**
> Brushless motors can spin up instantly and cause severe injury if props are mounted during testing or calibration.

---

## 2. Hardware Configuration

| ESC Identifier | ESP32 GPIO | Motor Location | Propeller Rotation |
| :--- | :--- | :--- | :--- |
| **ESC 1** | **GPIO 25** | Front-Right (FR) | Counter-Clockwise (**CCW**) |
| **ESC 2** | **GPIO 26** | Rear-Right (RR) | Clockwise (**CW**) |
| **ESC 3** | **GPIO 27** | Rear-Left (RL) | Counter-Clockwise (**CCW**) |
| **ESC 4** | **GPIO 14** (GPIO 33 on test sketch) | Front-Left (FL) | Clockwise (**CW**) |

- **PWM Frequency**: 250 Hz (or standard 50 Hz Servo PWM for initial bringup)
- **Pulse Duration**: $1000\,\mu\text{s}$ (idle/stop) to $2000\,\mu\text{s}$ (full throttle)

---

## 3. Interactive ESC Testing Tool (`esc_test.ino`)

The sketch located at [`esc_test/esc_test.ino`](../esc_test/esc_test.ino) provides an interactive command line interface over the serial monitor.

### Connecting to the Console:
```powershell
arduino-cli monitor -p COM3 --config baudrate=115200
```

### Supported CLI Commands:

| Command | Action Performed |
| :--- | :--- |
| `arm` | Sends $1000\,\mu\text{s}$ to all ESCs for 3 seconds to arm throttle safety |
| `test` | Spins each motor sequentially for 2 seconds at low throttle ($1150\,\mu\text{s}$) |
| `sweep` | Gradually ramps all 4 motors from $1000\,\mu\text{s}$ to $1300\,\mu\text{s}$ and back |
| `stop` | Instantly commands all motors back to $1000\,\mu\text{s}$ |
| `all <val>` | Sets all 4 ESCs to target microseconds (e.g., `all 1120`) |
| `1 <val>` | Sets ESC 1 target throttle in $\mu\text{s}$ |
| `2 <val>` | Sets ESC 2 target throttle in $\mu\text{s}$ |
| `3 <val>` | Sets ESC 3 target throttle in $\mu\text{s}$ |
| `4 <val>` | Sets ESC 4 target throttle in $\mu\text{s}$ |
| `status` | Reports current commanded pulse width on each channel |
| `help` | Prints the interactive command manual |

---

## 4. ESC Throttle Range Calibration (High-Low Calibration)

Standard ESCs must learn the transmitter's throttle endpoints:
1. Ensure props are **OFF**.
2. Flash calibration sketch sending $2000\,\mu\text{s}$ (Max).
3. Connect LiPo battery to ESCs. ESCs will emit a musical chime indicating high-throttle detection.
4. Immediately drop PWM to $1000\,\mu\text{s}$ (Min).
5. ESCs will beep confirming low-throttle calibration.
6. Disconnect battery. Throttle range is now saved in ESC EEPROM.
