# Ultrasonic Sensor (HC-SR04 / US-100) Usage & Calibration Guide

## 1. What to Use the Ultrasonic Sensor for on a Quadcopter

Ultrasonic rangefinders are indispensable for low-altitude flight regimes where barometers drift due to air pressure fluctuations or ground effect turbulence.

| Flight Feature | Description | Range Effective |
| :--- | :--- | :--- |
| **Precision Altitude Hold** | Maintains exact distance from floor/ground (e.g. 0.5m – 2.5m) | 2 cm – 350 cm |
| **Automated Smooth Landing** | Decelerates descent speed gracefully as ground approaches (< 30 cm) | 5 cm – 50 cm |
| **Ground-Effect Cushioning** | Compensates for the turbulent "air cushion" under the props near floor | 0 cm – 30 cm |
| **Forward Obstacle Avoidance**| When front-mounted, stops forward drift if a wall or obstacle is detected | 20 cm – 250 cm |

---

## 2. Electrical Wiring & Safety (5V to 3.3V)

Standard HC-SR04 modules run at 5V and output 5V logic pulses on the ECHO line. The ESP32 requires 3.3V logic on all input GPIOs.

### Wiring Diagram:
```text
HC-SR04            ESP32
───────            ─────
VCC   ──────────►  5V / VIN Rail
GND   ──────────►  GND (Shared)
TRIG  ──────────►  GPIO 18

ECHO  ───[ 1kΩ ]───┬───► GPIO 19
                   │
                [ 2kΩ ]
                   │
                  GND
```

*Note: If using 3.3V-compatible sensors (HC-SR04P, RCWL-1601, or US-100 in pulse mode), the voltage divider can be omitted.*

---

## 3. Calibration Procedure: Pre-Mount vs Post-Mount

### Should you calibrate before or after mounting?
- **Sensor Slope & Speed of Sound (`scaleMultiplier`)**: Can be calibrated on the bench at a known reference distance (e.g. 50.0 cm).
- **Ground Offset (`groundOffsetCm`)**: **MUST be calibrated AFTER mounting on the drone frame.**
  - When the quadcopter sits resting on its landing skids, the ultrasonic sensor will be elevated several centimeters above the ground (e.g., 4 cm – 8 cm).
  - The ground offset ensures that when the drone is on the ground, measured altitude reads `0.0 cm`.

---

## 4. Temperature Compensation Formula

The speed of sound in air varies with temperature:
$$v_{\text{sound}} = 331.3 + (0.606 \times T_{^\circ\text{C}})\quad [\text{m/s}]$$

For round-trip acoustic pulse measurement:
$$\text{Distance}\,(\text{cm}) = \frac{\text{EchoDuration}\,(\mu\text{s}) \times (0.03313 + 0.0000606 \times T)}{2} \times \text{scaleMultiplier} - \text{groundOffset}$$

The calibration utility in [`ultrasonic_calibrate/ultrasonic_calibrate.ino`](../ultrasonic_calibrate/ultrasonic_calibrate.ino) reads temperature automatically from the on-board MPU6050 internal thermometer over I2C.

---

## 5. Noise Rejection & Filtering in Flight

Due to acoustic noise from spinning propellers and motor vibrations:
1. **Median Filter**: Use a 5-to-15 sample median filter to discard stray acoustic echoes.
2. **Outlier Gate**: Discard sudden non-physical jumps ($\Delta h > 50\,\text{cm}$ in 20 ms).
3. **Tilt Compensation**:
   $$\text{Altitude}_{\text{true}} = \text{Distance}_{\text{measured}} \times \cos(\theta_{\text{pitch}}) \times \cos(\theta_{\text{roll}})$$
   When the drone tilts past $25^\circ$, ultrasonic pulses bounce away from the ground, so altitude hold should smoothly fall back to accelerometer integration.
