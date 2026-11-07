# Sensor Bench Telemetry & Validation Report

## 1. MPU6050 6-Axis IMU Verification

- **Bus**: I2C (`GPIO 21` SDA, `GPIO 22` SCL)
- **Clock**: 400 kHz Fast-Mode
- **Device ID (`WHO_AM_I` 0x75)**: `0x68`
- **Sampling Rate**: Deterministic 500 Hz

### Calibration Offsets Established:
```cpp
const float ACCEL_X_OFFSET = -863.19;
const float ACCEL_Y_OFFSET = -19.86;
const float ACCEL_Z_OFFSET = 12188.75;

const float GYRO_X_OFFSET  = -53.10;
const float GYRO_Y_OFFSET  = 97.94;
const float GYRO_Z_OFFSET  = 20.92;
```

---

## 2. ADXL345 3-Axis Accelerometer Verification

During hardware discovery on test bench:
- **I2C Address**: `0x53` (ALT ADDRESS tied low/floating)
- **Device ID (`DEVID` 0x00)**: `0xE5` (Analog Devices standard)
- **Operating Mode**: $\pm 4g$ Measurement Mode

### Sample Telemetry Stream:
```text
Streaming Live 3-Axis Acceleration:
   X-Axis       Y-Axis       Z-Axis     Total (g)
------------  ------------  ------------  ----------
0.094 g       0.998 g       0.023 g       1.003 g
0.094 g       0.998 g       0.023 g       1.003 g
0.086 g       1.006 g       0.023 g       1.010 g
0.094 g       1.006 g       0.031 g       1.011 g
0.101 g       1.006 g       0.023 g       1.012 g
```
- **Vector Sum Accuracy**: $1.003\,g$ (within 0.3% error of theoretical 1.000g local gravitational acceleration).
- **Signal Quality**: Low noise floor, immediate response to mechanical tilt.
