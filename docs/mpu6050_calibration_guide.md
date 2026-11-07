# MPU6050 Flight IMU Calibration Guide & Offsets

## 1. Calibration Overview

The InvenSense MPU6050 IMU combines a 3-axis accelerometer and a 3-axis angular rate sensor (gyroscope). To prevent steady-state drift and uncommanded attitude deviation during flight, accurate sensor zero-point offsets must be determined and subtracted in real-time.

- **Sensor Address**: `0x68` (AD0 tied to GND)
- **I2C Clock**: 400 kHz Fast-Mode
- **Sampling Iterations**: 2,000 samples averaged
- **Calibration Tool**: [`mpu_calibrate/mpu_calibrate.ino`](../mpu_calibrate/mpu_calibrate.ino)

---

## 2. Saved Calibration Offsets

The following values were generated during empirical calibration with the sensor resting flat and stationary:

```cpp
// -------------------------------------------------------------
// MPU6050 Calibrated Offsets (Generated via 2,000-sample run)
// -------------------------------------------------------------
const float ACCEL_X_OFFSET = -863.19f;
const float ACCEL_Y_OFFSET = -19.86f;
const float ACCEL_Z_OFFSET = 12188.75f;

const float GYRO_X_OFFSET  = -53.10f;
const float GYRO_Y_OFFSET  = 97.94f;
const float GYRO_Z_OFFSET  = 20.92f;
```

---

## 3. Mathematical Conversion to Physical Units

MPU6050 registers yield signed 16-bit integers (`-32768` to `+32767`). With the default configuration ($\pm 2g$ accelerometer full-scale, $\pm 250^\circ/\text{s}$ gyroscope full-scale):

- **Accelerometer Sensitivity**: $16,384\,\text{LSB} / g$
- **Gyroscope Sensitivity**: $131.0\,\text{LSB} / (^\circ/\text{s})$

### Applying Offsets in Firmware:
```cpp
// Raw signed 16-bit register reads:
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;

// Acceleration in g:
float ax_g = (raw_ax - ACCEL_X_OFFSET) / 16384.0f;
float ay_g = (raw_ay - ACCEL_Y_OFFSET) / 16384.0f;
float az_g = (raw_az - ACCEL_Z_OFFSET) / 16384.0f;

// Angular velocity in deg/s:
float gx_dps = (raw_gx - GYRO_X_OFFSET) / 131.0f;
float gy_dps = (raw_gy - GYRO_Y_OFFSET) / 131.0f;
float gz_dps = (raw_gz - GYRO_Z_OFFSET) / 131.0f;
```

### Upright Target Readings:
When the drone is level on the ground:
- $a_x \approx 0.00\,g$
- $a_y \approx 0.00\,g$
- $a_z \approx +1.00\,g$
- $\omega_x \approx 0.00\,^\circ/\text{s}$
- $\omega_y \approx 0.00\,^\circ/\text{s}$
- $\omega_z \approx 0.00\,^\circ/\text{s}$

---

## 4. Re-Calibrating the IMU

If the sensor is remounted or experiencing drift:
1. Place the quadcopter on an absolutely flat, level surface with motors powered off.
2. Upload `mpu_calibrate/mpu_calibrate.ino` to the ESP32.
3. Open the Serial Monitor at 115200 baud.
4. Keep the frame completely still until all 2,000 samples are collected.
5. Copy the newly printed offset constants into `flight_controller/flight_controller.ino`.
