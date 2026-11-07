/*
 * Universal IMU Diagnostic & Face-Down Calibration
 */

#include <Wire.h>

#define PIN_A 21
#define PIN_B 22

uint8_t sdaPin = 21;
uint8_t sclPin = 22;
uint8_t activeAddr = 0;
bool isADXL = false;

const int CALIB_SAMPLES = 2000;

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println(F("\n======================================================="));
  Serial.println(F("       IMU Bus Health & Auto-Calibration Check         "));
  Serial.println(F("======================================================="));

  // 1. Check physical voltage levels of GPIO 21 and 22
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  delay(50);

  int vA = digitalRead(PIN_A);
  int vB = digitalRead(PIN_B);

  Serial.println(F("[1] Physical Pin Voltage Check:"));
  Serial.print(F("    GPIO 21: ")); Serial.println(vA ? F("HIGH (OK)") : F("LOW (GROUNDED / SHORTED!)"));
  Serial.print(F("    GPIO 22: ")); Serial.println(vB ? F("HIGH (OK)") : F("LOW (GROUNDED / SHORTED!)"));

  if (!vA || !vB) {
    Serial.println(F("\n[!] CRITICAL: At least one I2C pin is pulled to GND!"));
    Serial.println(F("    Possible causes:"));
    Serial.println(F("    - When turning sensor face-down, a wire touched GND or pulled out."));
    Serial.println(F("    - Check if the power LED on the sensor is lit."));
    while(1) delay(1000);
  }

  // 2. Scan on default pinout (SDA=21, SCL=22)
  Serial.println(F("\n[2] Scanning I2C bus (SDA=21, SCL=22)..."));
  Wire.begin(21, 22, 100000);
  Wire.setTimeOut(50);

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("    --> Found device at 0x"));
      Serial.println(addr, HEX);
      activeAddr = addr;
      sdaPin = 21;
      sclPin = 22;
      break;
    }
  }

  // 3. If nothing found, try swapped pinout (SDA=22, SCL=21)
  if (activeAddr == 0) {
    Serial.println(F("    [No device on 21/22]. Testing swapped pins (SDA=22, SCL=21)..."));
    Wire.end();
    delay(50);
    Wire.begin(22, 21, 100000);
    Wire.setTimeOut(50);

    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print(F("    --> Found device at 0x"));
        Serial.println(addr, HEX);
        activeAddr = addr;
        sdaPin = 22;
        sclPin = 21;
        break;
      }
    }
  }

  if (activeAddr == 0) {
    Serial.println(F("\n[FAIL] No I2C device detected on either pin combination!"));
    Serial.println(F("Checklist:"));
    Serial.println(F("  1. Is the small red/green LED on the sensor glowing right now?"));
    Serial.println(F("  2. Did a jumper wire become loose when flipping it face down?"));
    while (1) delay(1000);
  }

  // 4. Identify Device
  if (activeAddr == 0x53) {
    isADXL = true;
    Serial.println(F("\n>>> Identified as ADXL345 Accelerometer (0x53) <<<"));
    // Measurement mode
    Wire.beginTransmission(0x53);
    Wire.write(0x2D);
    Wire.write(0x08);
    Wire.endTransmission();
    // +-4g mode
    Wire.beginTransmission(0x53);
    Wire.write(0x31);
    Wire.write(0x01);
    Wire.endTransmission();
  } else {
    Serial.print(F("\n>>> Identified as MPU6050 (0x"));
    Serial.print(activeAddr, HEX);
    Serial.println(F(") <<<"));
    // Wake up MPU
    Wire.beginTransmission(activeAddr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
  }

  Serial.println(F("\nSensor is resting still."));
  Serial.println(F("Starting 2000-sample calibration in 2 seconds..."));
  delay(2000);

  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

    if (isADXL) {
      Wire.beginTransmission(0x53);
      Wire.write(0x32);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x53, (uint8_t)6) == 6) {
        ax = Wire.read() | (Wire.read() << 8);
        ay = Wire.read() | (Wire.read() << 8);
        az = Wire.read() | (Wire.read() << 8);
      }
    } else {
      Wire.beginTransmission(activeAddr);
      Wire.write(0x3B);
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)activeAddr, (uint8_t)14) == 14) {
        ax = (Wire.read() << 8) | Wire.read();
        ay = (Wire.read() << 8) | Wire.read();
        az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read();
        gx = (Wire.read() << 8) | Wire.read();
        gy = (Wire.read() << 8) | Wire.read();
        gz = (Wire.read() << 8) | Wire.read();
      }
    }

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;

    if (i % 500 == 0) {
      Serial.print(F("  Progress: "));
      Serial.print((i * 100) / CALIB_SAMPLES);
      Serial.println(F("%"));
    }
    delay(2);
  }

  float ax_off = (float)sum_ax / CALIB_SAMPLES;
  float ay_off = (float)sum_ay / CALIB_SAMPLES;
  float az_off;
  if (isADXL) {
    // ADXL345 at +-4g is ~128 LSB/g. In face down, Z reads -128.
    az_off = ((float)sum_az / CALIB_SAMPLES) + 128.0;
  } else {
    // MPU6050 at +-2g is 16384 LSB/g. In face down, Z reads -16384.
    az_off = ((float)sum_az / CALIB_SAMPLES) + 16384.0;
  }

  float gx_off = (float)sum_gx / CALIB_SAMPLES;
  float gy_off = (float)sum_gy / CALIB_SAMPLES;
  float gz_off = (float)sum_gz / CALIB_SAMPLES;

  Serial.println(F("\n======================================================="));
  Serial.println(F("              CALIBRATION COMPLETE!                    "));
  Serial.println(F("======================================================="));
  Serial.println(F("// Copy and paste these constants into your flight code:"));
  Serial.print(F("const float ACCEL_X_OFFSET = ")); Serial.print(ax_off, 2); Serial.println(F(";"));
  Serial.print(F("const float ACCEL_Y_OFFSET = ")); Serial.print(ay_off, 2); Serial.println(F(";"));
  Serial.print(F("const float ACCEL_Z_OFFSET = ")); Serial.print(az_off, 2); Serial.println(F(";"));
  if (!isADXL) {
    Serial.print(F("const float GYRO_X_OFFSET  = ")); Serial.print(gx_off, 2); Serial.println(F(";"));
    Serial.print(F("const float GYRO_Y_OFFSET  = ")); Serial.print(gy_off, 2); Serial.println(F(";"));
    Serial.print(F("const float GYRO_Z_OFFSET  = ")); Serial.print(gz_off, 2); Serial.println(F(";"));
  }
  Serial.println(F("=======================================================\n"));

  Serial.println(F("Calibrated Real-Time Stream (Flip face-up to verify Z = +1.00g!):"));
}

void loop() {
  delay(500);
}
