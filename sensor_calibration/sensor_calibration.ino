/*
 * Sensor Calibration Sketch for Arduino Uno
 * -----------------------------------------
 * Wiring:
 *   HC-SR04:
 *     VCC  -> 5V
 *     TRIG -> D6
 *     ECHO -> D7
 *     GND  -> GND
 *
 *   GY-521 (MPU-6050):
 *     VCC  -> 3.3V (or 5V if 3.3V causes brownout on clone boards)
 *     GND  -> GND
 *     SDA  -> A4
 *     SCL  -> A5
 *     INT  -> Not connected
 *     AD0  -> Not connected (Address 0x68)
 *
 * Instructions:
 *   1. Keep the GY-521 completely STILL and flat on a table during startup.
 *   2. Serial monitor set to 115200 baud.
 */

#include <Wire.h>

// Pins
const int TRIG_PIN = 6;
const int ECHO_PIN = 7;

// MPU6050 I2C
const uint8_t MPU6050_ADDR = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_CONFIG     = 0x1A;
const uint8_t REG_ACCEL_XOUT = 0x3B;

// Calibration offsets
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

// HC-SR04 offset (mounting distance adjustment in cm)
float ultrasonic_offset_cm = 0.0;

// Number of samples for calibration
const int CALIB_SAMPLES = 1000;

void readMPURaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &tempRaw,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() >= 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    tempRaw = (Wire.read() << 8) | Wire.read();
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

float readUltrasonicDistanceCm(float tempC) {
  // Clear trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send 10us trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure round-trip time in microseconds (timeout at 30ms ~ 5 meters)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    return -1.0; // Out of range or no echo received
  }

  // Speed of sound compensated for temperature:
  // v = 331.3 + 0.606 * Temp(°C) [m/s]
  // In cm/us: v * 100 / 1,000,000 = (331.3 + 0.606 * Temp) * 0.0001
  float soundSpeedCmPerUs = (331.3 + 0.606 * tempC) * 0.0001;

  // One-way distance = (duration / 2) * speed
  float distance = (duration / 2.0) * soundSpeedCmPerUs + ultrasonic_offset_cm;
  return distance;
}

void calibrateMPU() {
  Serial.println(F("\n==========================================="));
  Serial.println(F(" CALIBRATING MPU6050..."));
  Serial.println(F(" >> DO NOT MOVE OR VIBRATE THE SENSOR! <<"));
  Serial.println(F(" >> Ensure sensor is resting FLAT (Z UP) <<"));
  Serial.println(F("==========================================="));

  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  // Discard first 100 readings for sensor stabilization
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, t, gx, gy, gz;
    readMPURaw(ax, ay, az, t, gx, gy, gz);
    delay(2);
  }

  // Accumulate samples
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    int16_t ax, ay, az, t, gx, gy, gz;
    readMPURaw(ax, ay, az, t, gx, gy, gz);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    if (i % 200 == 0) {
      Serial.print(F("Progress: "));
      Serial.print((i * 100) / CALIB_SAMPLES);
      Serial.println(F("%"));
    }
    delay(3);
  }

  // Calculate averages
  ax_offset = (float)ax_sum / CALIB_SAMPLES;
  ay_offset = (float)ay_sum / CALIB_SAMPLES;
  // At rest on flat surface, 1g (+16384 LSB at default ±2g) acts on Z
  az_offset = ((float)az_sum / CALIB_SAMPLES) - 16384.0;

  gx_offset = (float)gx_sum / CALIB_SAMPLES;
  gy_offset = (float)gy_sum / CALIB_SAMPLES;
  gz_offset = (float)gz_sum / CALIB_SAMPLES;

  Serial.println(F("\n--- Calibration Complete! ---"));
  Serial.print(F("Accel Offsets (LSB):  X=")); Serial.print(ax_offset, 1);
  Serial.print(F("  Y=")); Serial.print(ay_offset, 1);
  Serial.print(F("  Z=")); Serial.println(az_offset, 1);

  Serial.print(F("Gyro Offsets (LSB):   X=")); Serial.print(gx_offset, 1);
  Serial.print(F("  Y=")); Serial.print(gy_offset, 1);
  Serial.print(F("  Z=")); Serial.println(gz_offset, 1);
  Serial.println(F("-----------------------------\n"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // Setup Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz fast I2C mode

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00); // Clear sleep bit
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.println(F("[ERROR] GY-521 (MPU6050) not detected at 0x68!"));
    Serial.println(F("Check SDA(A4), SCL(A5), VCC, GND connections."));
    while (1) { delay(1000); }
  }

  // Enable Low-Pass Filter (DLPF ~42Hz bandwidth) to reduce vibration noise
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_CONFIG);
  Wire.write(0x03);
  Wire.endTransmission();

  delay(200);

  // Perform calibration routine
  calibrateMPU();

  Serial.println(F("Streaming calibrated data:"));
  Serial.println(F("Roll(deg/s)\tPitch(deg/s)\tYaw(deg/s)\tAx(g)\tAy(g)\tAz(g)\tTemp(C)\tDist(cm)"));
}

void loop() {
  int16_t rawAx, rawAy, rawAz, rawTemp, rawGx, rawGy, rawGz;
  readMPURaw(rawAx, rawAy, rawAz, rawTemp, rawGx, rawGy, rawGz);

  // Apply calibration offsets
  float calAx = (rawAx - ax_offset) / 16384.0; // in g
  float calAy = (rawAy - ay_offset) / 16384.0; // in g
  float calAz = (rawAz - az_offset) / 16384.0; // in g

  // Default gyro scale: ±250 deg/s -> 131.0 LSB / (deg/s)
  float calGx = (rawGx - gx_offset) / 131.0;   // in deg/s
  float calGy = (rawGy - gy_offset) / 131.0;   // in deg/s
  float calGz = (rawGz - gz_offset) / 131.0;   // in deg/s

  float tempC = (rawTemp / 340.0) + 36.53;

  // Read temperature-compensated ultrasonic distance
  float distanceCm = readUltrasonicDistanceCm(tempC);

  // Print results
  Serial.print(calGx, 2);   Serial.print('\t');
  Serial.print(calGy, 2);   Serial.print('\t');
  Serial.print(calGz, 2);   Serial.print('\t');
  Serial.print(calAx, 2);   Serial.print('\t');
  Serial.print(calAy, 2);   Serial.print('\t');
  Serial.print(calAz, 2);   Serial.print('\t');
  Serial.print(tempC, 1);   Serial.print('\t');
  if (distanceCm < 0) {
    Serial.println(F("OUT_OF_RANGE"));
  } else {
    Serial.println(distanceCm, 1);
  }

  delay(100);
}
