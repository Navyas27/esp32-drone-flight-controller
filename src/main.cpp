/*
 * =========================================================================
 * ESP32 Quadcopter Flight Controller Firmware
 * =========================================================================
 * Framework   : Arduino / PlatformIO
 * Target      : ESP32 Dev Module
 * Architecture: Cascaded Angle + Rate PID running at deterministic 500 Hz
 * Control     : ESP-NOW wireless glove link + 500 ms failsafe auto-disarm
 * Safety      : Disarmed on boot (1000us), 2s low-throttle arm interlock,
 *               500ms packet timeout failsafe, Serial 'k' emergency kill
 * =========================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// =========================================================================
// 1. PIN CONFIGURATION
// =========================================================================
// I2C Bus for MPU6050
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define I2C_CLOCK_SPEED 400000  // 400 kHz Fast-Mode I2C

// ESC Motor Output Pins (Quad-X Configuration)
#define PIN_MOTOR_1     25  // M1: Front-Right (CCW)
#define PIN_MOTOR_2     26  // M2: Rear-Right  (CW)
#define PIN_MOTOR_3     27  // M3: Rear-Left   (CCW)
#define PIN_MOTOR_4     14  // M4: Front-Left  (CW)

#define ESC_PWM_FREQ_HZ 250 // 250 Hz standard 1000-2000us ESC update rate

// =========================================================================
// 2. PID TUNING CONSTANTS (Conservative Defaults)
// =========================================================================
// Outer Loop: Angle P-Controller (Angle Error [deg] -> Target Rate [deg/s])
const float ANGLE_KP_ROLL     = 4.0f;
const float ANGLE_KP_PITCH    = 4.0f;

// Inner Loop: Rate PID Controllers (Rate Error [deg/s] -> PWM Offset [us])
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

// Integral Anti-Windup Clamping (Max I-term contribution in microseconds)
const float PID_I_LIMIT_ROLL  = 120.0f;
const float PID_I_LIMIT_PITCH = 120.0f;
const float PID_I_LIMIT_YAW   = 100.0f;

// Max Total PID Output Clamp per Axis (microseconds)
const float PID_OUT_LIMIT     = 350.0f;

// Complementary Filter Coefficient (98.5% Gyro + 1.5% Accel at 500 Hz)
const float COMP_FILTER_ALPHA = 0.985f;

// Maximum Control Setpoints from Glove
const float MAX_TARGET_ANGLE_DEG = 30.0f;   // Max roll/pitch tilt (deg)
const float MAX_TARGET_YAW_RATE  = 150.0f;  // Max yaw rotation rate (deg/s)

// =========================================================================
// 3. SAFETY & LOOP TIMING CONSTANTS
// =========================================================================
const uint32_t LOOP_TIME_US        = 2000;  // Fixed 500 Hz control loop (2000 us)
const uint32_t FAILSAFE_TIMEOUT_MS = 500;   // Auto-disarm if no ESP-NOW packet for 500 ms
const uint32_t ARM_HOLD_TIME_MS    = 2000;  // Throttle low for 2000 ms to arm
const uint32_t TELEMETRY_PERIOD_MS = 100;   // 10 Hz low-rate Serial debug output

const int16_t  THROTTLE_MIN        = 1000;  // Disarmed / zero motor pulse (us)
const int16_t  THROTTLE_ARM_MAX    = 1050;  // Max throttle allowed during 2s arming
const int16_t  THROTTLE_IDLE       = 1060;  // Armed idle pulse (us)
const int16_t  THROTTLE_MAX        = 2000;  // Full power pulse (us)

// MPU6050 Registers & Scale Factors
#define MPU_ADDR_PRIMARY   0x68
#define MPU_ADDR_SECONDARY 0x69
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_PWR_MGMT_1     0x6B
#define REG_ACCEL_XOUT_H   0x3B

const float ACCEL_LSB_PER_G    = 8192.0f;   // +-4g full scale
const float GYRO_LSB_PER_DPS   = 65.5f;     // +-500 deg/s full scale

// =========================================================================
// 4. DATA STRUCTURES & STATE VARIABLES
// =========================================================================
// Incoming ESP-NOW command packet from Glove ESP32
typedef struct __attribute__((packed)) {
  int16_t roll;      // Roll command:  -500..+500 -> -30..+30 deg (or direct deg if -30..+30)
  int16_t pitch;     // Pitch command: -500..+500 -> -30..+30 deg
  int16_t yaw_rate;  // Yaw rate cmd:  -500..+500 -> -150..+150 deg/s
  int16_t throttle;  // Throttle PWM:  1000..2000 us
} GloveCommand;

volatile GloveCommand rxCommand = {0, 0, 0, 1000};
volatile uint32_t lastPacketMs  = 0;
volatile bool hasReceivedPacket = false;

Servo esc1, esc2, esc3, esc4;

uint8_t  mpuAddr     = MPU_ADDR_PRIMARY;
uint8_t  activeSda   = PIN_I2C_SDA;
uint8_t  activeScl   = PIN_I2C_SCL;
uint32_t activeI2cHz = I2C_CLOCK_SPEED;
bool     wireStarted = false;

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float rollAngle = 0.0f, pitchAngle = 0.0f;

// Last valid raw IMU sample (protects against transient I2C NACKs)
int16_t lastRawAx = 0, lastRawAy = 0, lastRawAz = 8192;
int16_t lastRawGx = 0, lastRawGy = 0, lastRawGz = 0;

// PID integrators & previous rate errors
float iTermRoll  = 0.0f, prevRateErrRoll  = 0.0f;
float iTermPitch = 0.0f, prevRateErrPitch = 0.0f;
float iTermYaw   = 0.0f, prevRateErrYaw   = 0.0f;

enum FlightState { STATE_DISARMED, STATE_ARMED, STATE_KILLED };
FlightState flightState = STATE_DISARMED;

uint32_t armLowStartMs   = 0;
uint32_t lastLoopUs      = 0;
uint32_t lastTelemetryMs = 0;

int m1Pwm = THROTTLE_MIN, m2Pwm = THROTTLE_MIN, m3Pwm = THROTTLE_MIN, m4Pwm = THROTTLE_MIN;

// =========================================================================
// 5. ESP-NOW RECEIVE CALLBACK
// =========================================================================
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
  if (len == (int)sizeof(GloveCommand)) {
    memcpy((void *)&rxCommand, incomingData, sizeof(GloveCommand));
    lastPacketMs = millis();
    hasReceivedPacket = true;
  }
}

// =========================================================================
// 6. MOTOR OUTPUT & RESET HELPERS
// =========================================================================
void resetPID() {
  iTermRoll = 0.0f;  prevRateErrRoll = 0.0f;
  iTermPitch = 0.0f; prevRateErrPitch = 0.0f;
  iTermYaw = 0.0f;   prevRateErrYaw = 0.0f;
}

void writeMotors(int m1, int m2, int m3, int m4) {
  if (flightState != STATE_ARMED) {
    m1 = THROTTLE_MIN;
    m2 = THROTTLE_MIN;
    m3 = THROTTLE_MIN;
    m4 = THROTTLE_MIN;
  }
  m1Pwm = constrain(m1, (int)THROTTLE_MIN, (int)THROTTLE_MAX);
  m2Pwm = constrain(m2, (int)THROTTLE_MIN, (int)THROTTLE_MAX);
  m3Pwm = constrain(m3, (int)THROTTLE_MIN, (int)THROTTLE_MAX);
  m4Pwm = constrain(m4, (int)THROTTLE_MIN, (int)THROTTLE_MAX);

  esc1.writeMicroseconds(m1Pwm);
  esc2.writeMicroseconds(m2Pwm);
  esc3.writeMicroseconds(m3Pwm);
  esc4.writeMicroseconds(m4Pwm);
}

// =========================================================================
// 7. MPU6050 DRIVER & BOOT GYRO CALIBRATION
// =========================================================================
bool writeMPUReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) == 0 &&
      Wire.requestFrom((uint8_t)mpuAddr, (uint8_t)14) == 14) {
    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());
    Wire.read(); Wire.read(); // Discard temperature bytes
    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());

    lastRawAx = ax; lastRawAy = ay; lastRawAz = az;
    lastRawGx = gx; lastRawGy = gy; lastRawGz = gz;
    return true;
  }
  // Fallback to last valid reading if a single I2C transfer glitches
  ax = lastRawAx; ay = lastRawAy; az = lastRawAz;
  gx = lastRawGx; gy = lastRawGy; gz = lastRawGz;
  return false;
}

bool tryInitMPUOnPins(uint8_t sda, uint8_t scl) {
  if (wireStarted) {
    Wire.end();
    delay(20);
  }
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delay(20);

  Wire.begin(sda, scl, 100000); // Start at 100 kHz for reliable wake-up
  wireStarted = true;
  Wire.setTimeOut(20);
  delay(30);

  const uint8_t addrs[2] = {MPU_ADDR_PRIMARY, MPU_ADDR_SECONDARY};
  for (int i = 0; i < 2; i++) {
    uint8_t addr = addrs[i];
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      // Wake up MPU6050 and configure DLPF + ranges
      if (!writeMPUReg(addr, REG_PWR_MGMT_1, 0x00)) continue;
      delay(30);
      writeMPUReg(addr, REG_CONFIG, 0x03);       // ~44 Hz hardware DLPF
      writeMPUReg(addr, REG_GYRO_CONFIG, 0x08);  // +-500 deg/s (65.5 LSB/dps)
      writeMPUReg(addr, REG_ACCEL_CONFIG, 0x08); // +-4g (8192 LSB/g)

      mpuAddr = addr;
      int16_t ax, ay, az, gx, gy, gz;

      // Switch to requested 400 kHz Fast-Mode I2C and verify burst read
      Wire.setClock(I2C_CLOCK_SPEED);
      delay(10);
      if (readMPU(ax, ay, az, gx, gy, gz)) {
        activeSda   = sda;
        activeScl   = scl;
        activeI2cHz = I2C_CLOCK_SPEED;
        return true;
      }

      // Graceful fallback to 100 kHz if long jumper wires NACK at 400 kHz
      Wire.setClock(100000);
      delay(10);
      if (readMPU(ax, ay, az, gx, gy, gz)) {
        activeSda   = sda;
        activeScl   = scl;
        activeI2cHz = 100000;
        return true;
      }
    }
  }
  return false;
}

bool initMPU() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    // 1. Standard pins: SDA = GPIO 21, SCL = GPIO 22
    if (tryInitMPUOnPins(PIN_I2C_SDA, PIN_I2C_SCL)) {
      Serial.printf("[IMU]   MPU6050 ready at 0x%02X (SDA=%d, SCL=%d, %lu kHz)\n",
                    mpuAddr, activeSda, activeScl, (unsigned long)(activeI2cHz / 1000));
      return true;
    }
    // 2. Auto-detect fallback if SDA/SCL wires are swapped (SDA = 22, SCL = 21)
    if (tryInitMPUOnPins(PIN_I2C_SCL, PIN_I2C_SDA)) {
      Serial.printf("[IMU]   MPU6050 ready at 0x%02X (Swapped wires: SDA=%d, SCL=%d, %lu kHz)\n",
                    mpuAddr, activeSda, activeScl, (unsigned long)(activeI2cHz / 1000));
      return true;
    }
    delay(50);
  }
  return false;
}

void checkSerialKill() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'k' || c == 'K') {
      flightState = STATE_KILLED;
      resetPID();
      writeMotors(THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN);
      Serial.println(F("\n[KILL] Serial 'k' received -> MOTORS KILLED (1000us)!"));
    }
  }
}

void calibrateGyro() {
  Serial.println(F("[CALIB] Calibrating Gyroscope (keep drone still)..."));
  const int SAMPLES = 1000;
  double sumGx = 0.0, sumGy = 0.0, sumGz = 0.0;
  double sumAx = 0.0, sumAy = 0.0, sumAz = 0.0;

  // Discard initial settling samples
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU(ax, ay, az, gx, gy, gz);
    delay(2);
  }

  for (int i = 0; i < SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU(ax, ay, az, gx, gy, gz);
    sumGx += gx; sumGy += gy; sumGz += gz;
    sumAx += ax; sumAy += ay; sumAz += az;
    delay(2);
  }

  gyroBiasX = (float)(sumGx / SAMPLES);
  gyroBiasY = (float)(sumGy / SAMPLES);
  gyroBiasZ = (float)(sumGz / SAMPLES);

  // Seed initial complementary filter angles from gravity vector
  float axG = (float)(sumAx / SAMPLES) / ACCEL_LSB_PER_G;
  float ayG = (float)(sumAy / SAMPLES) / ACCEL_LSB_PER_G;
  float azG = (float)(sumAz / SAMPLES) / ACCEL_LSB_PER_G;
  rollAngle  = atan2f(ayG, azG) * RAD_TO_DEG;
  pitchAngle = atan2f(-axG, sqrtf(ayG * ayG + azG * azG)) * RAD_TO_DEG;

  Serial.printf("[CALIB] Gyro Bias (LSB) -> X:%.2f Y:%.2f Z:%.2f | Init Roll:%.1f deg Pitch:%.1f deg\n",
                gyroBiasX, gyroBiasY, gyroBiasZ, rollAngle, pitchAngle);
}

// =========================================================================
// 8. SYSTEM SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println(F("\n======================================================="));
  Serial.println(F("         ESP32 QUADCOPTER FLIGHT CONTROLLER            "));
  Serial.println(F("======================================================="));

  // 1. Immediately lock all 4 ESC outputs to 1000us (Disarmed Safe State)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.setPeriodHertz(ESC_PWM_FREQ_HZ);
  esc2.setPeriodHertz(ESC_PWM_FREQ_HZ);
  esc3.setPeriodHertz(ESC_PWM_FREQ_HZ);
  esc4.setPeriodHertz(ESC_PWM_FREQ_HZ);

  esc1.attach(PIN_MOTOR_1, THROTTLE_MIN, THROTTLE_MAX);
  esc2.attach(PIN_MOTOR_2, THROTTLE_MIN, THROTTLE_MAX);
  esc3.attach(PIN_MOTOR_3, THROTTLE_MIN, THROTTLE_MAX);
  esc4.attach(PIN_MOTOR_4, THROTTLE_MIN, THROTTLE_MAX);

  writeMotors(THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN);
  Serial.println(F("[MOTOR] M1(25) M2(26) M3(27) M4(14) locked at 1000us (DISARMED)"));

  // 2. Initialize ESP-NOW Receiver
  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  uint8_t staMac[6] = {0};
  esp_read_mac(staMac, ESP_MAC_WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[FATAL] ESP-NOW init failed!"));
    while (1) { delay(1000); }
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.printf("[WIFI]  ESP-NOW Ready | Drone MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                staMac[0], staMac[1], staMac[2], staMac[3], staMac[4], staMac[5]);

  // 3. Initialize MPU6050 (retries cleanly while keeping motors at 1000us)
  while (!initMPU()) {
    checkSerialKill();
    if (wireStarted) {
      Wire.end();
      wireStarted = false;
    }
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    delay(10);
    Serial.printf("[WARN]  MPU6050 not detected (GPIO21=%d, GPIO22=%d). Retrying in 1s...\n",
                  digitalRead(PIN_I2C_SDA), digitalRead(PIN_I2C_SCL));
    writeMotors(THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN);
    for (int w = 0; w < 20; w++) {
      checkSerialKill();
      delay(50);
    }
  }

  // 4. Boot Gyroscope Calibration
  calibrateGyro();

  Serial.println(F("[READY] 500 Hz Control Loop Active."));
  Serial.println(F("        Arm: Receive ESP-NOW packets with throttle <= 1050us for 2.0s."));
  Serial.println(F("        Kill: Send 'k' over Serial at any time.\n"));

  lastLoopUs = micros();
  lastTelemetryMs = millis();
}

// =========================================================================
// 9. FIXED 500 HZ FLIGHT CONTROL LOOP
// =========================================================================
float mapAxisSetpoint(int16_t rawVal, float maxLimit) {
  // Supports both normalized (-500..+500) and direct degree/rate commands
  if (abs(rawVal) <= (int16_t)maxLimit) {
    return constrain((float)rawVal, -maxLimit, maxLimit);
  }
  return constrain((float)rawVal * (maxLimit / 500.0f), -maxLimit, maxLimit);
}

void loop() {
  // Always check Serial 'k' kill switch immediately without waiting
  checkSerialKill();

  // Enforce fixed 500 Hz loop timing (2000 us period)
  uint32_t nowUs = micros();
  if ((nowUs - lastLoopUs) < LOOP_TIME_US) {
    return;
  }
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;
  dt = constrain(dt, 0.0005f, 0.010f); // Guard against timer anomalies

  // Snapshot volatile ESP-NOW command atomically
  noInterrupts();
  GloveCommand cmd = *(const GloveCommand *)&rxCommand;
  uint32_t pktMs   = lastPacketMs;
  bool gotPkt      = hasReceivedPacket;
  interrupts();

  uint32_t nowMs = millis();
  bool linkTimedOut = (!gotPkt) || ((nowMs - pktMs) > FAILSAFE_TIMEOUT_MS);

  // -----------------------------------------------------------------------
  // Safety Check 1: Auto-Disarm if no ESP-NOW packet for > 500 ms
  // -----------------------------------------------------------------------
  if (linkTimedOut) {
    armLowStartMs = 0;
    if (flightState == STATE_ARMED) {
      flightState = STATE_DISARMED;
      resetPID();
      writeMotors(THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN);
      Serial.println(F("[FAILSAFE] No ESP-NOW packet > 500ms -> AUTO-DISARMED!"));
    }
  }

  // -----------------------------------------------------------------------
  // Safety Check 2: Arm Condition (Throttle low <= 1050us continuously for 2s)
  // -----------------------------------------------------------------------
  if (flightState == STATE_DISARMED && !linkTimedOut) {
    if (cmd.throttle >= (THROTTLE_MIN - 50) && cmd.throttle <= THROTTLE_ARM_MAX) {
      if (armLowStartMs == 0) {
        armLowStartMs = nowMs;
      } else if ((nowMs - armLowStartMs) >= ARM_HOLD_TIME_MS) {
        flightState = STATE_ARMED;
        armLowStartMs = 0;
        resetPID();
        Serial.println(F("[ARMED] Low throttle held for 2s -> MOTORS ARMED!"));
      }
    } else {
      armLowStartMs = 0;
    }
  }

  // -----------------------------------------------------------------------
  // IMU Read & Complementary Filter (Roll / Pitch Angle Estimation)
  // -----------------------------------------------------------------------
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  readMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

  float axG = (float)rawAx / ACCEL_LSB_PER_G;
  float ayG = (float)rawAy / ACCEL_LSB_PER_G;
  float azG = (float)rawAz / ACCEL_LSB_PER_G;

  float gxDps = ((float)rawGx - gyroBiasX) / GYRO_LSB_PER_DPS;
  float gyDps = ((float)rawGy - gyroBiasY) / GYRO_LSB_PER_DPS;
  float gzDps = ((float)rawGz - gyroBiasZ) / GYRO_LSB_PER_DPS;

  float accRoll  = atan2f(ayG, azG) * RAD_TO_DEG;
  float accPitch = atan2f(-axG, sqrtf(ayG * ayG + azG * azG)) * RAD_TO_DEG;

  rollAngle  = COMP_FILTER_ALPHA * (rollAngle  + gxDps * dt) + (1.0f - COMP_FILTER_ALPHA) * accRoll;
  pitchAngle = COMP_FILTER_ALPHA * (pitchAngle + gyDps * dt) + (1.0f - COMP_FILTER_ALPHA) * accPitch;

  // -----------------------------------------------------------------------
  // Cascaded Attitude + Rate PID & Quad-X Motor Mixing
  // -----------------------------------------------------------------------
  int outM1 = THROTTLE_MIN, outM2 = THROTTLE_MIN, outM3 = THROTTLE_MIN, outM4 = THROTTLE_MIN;

  if (flightState == STATE_ARMED) {
    int baseThrottle = constrain((int)cmd.throttle, (int)THROTTLE_MIN, (int)THROTTLE_MAX);

    if (baseThrottle >= THROTTLE_IDLE) {
      float targetRoll    = mapAxisSetpoint(cmd.roll, MAX_TARGET_ANGLE_DEG);
      float targetPitch   = mapAxisSetpoint(cmd.pitch, MAX_TARGET_ANGLE_DEG);
      float targetYawRate = mapAxisSetpoint(cmd.yaw_rate, MAX_TARGET_YAW_RATE);

      // Outer Loop (Angle P -> Desired Rate in deg/s)
      float desiredRollRate  = ANGLE_KP_ROLL  * (targetRoll  - rollAngle);
      float desiredPitchRate = ANGLE_KP_PITCH * (targetPitch - pitchAngle);

      // Inner Loop: Roll Rate PID
      float errRoll = desiredRollRate - gxDps;
      iTermRoll = constrain(iTermRoll + RATE_KI_ROLL * errRoll * dt, -PID_I_LIMIT_ROLL, PID_I_LIMIT_ROLL);
      float dRoll = RATE_KD_ROLL * (errRoll - prevRateErrRoll) / dt;
      prevRateErrRoll = errRoll;
      float pidRoll = constrain(RATE_KP_ROLL * errRoll + iTermRoll + dRoll, -PID_OUT_LIMIT, PID_OUT_LIMIT);

      // Inner Loop: Pitch Rate PID
      float errPitch = desiredPitchRate - gyDps;
      iTermPitch = constrain(iTermPitch + RATE_KI_PITCH * errPitch * dt, -PID_I_LIMIT_PITCH, PID_I_LIMIT_PITCH);
      float dPitch = RATE_KD_PITCH * (errPitch - prevRateErrPitch) / dt;
      prevRateErrPitch = errPitch;
      float pidPitch = constrain(RATE_KP_PITCH * errPitch + iTermPitch + dPitch, -PID_OUT_LIMIT, PID_OUT_LIMIT);

      // Yaw Rate PID
      float errYaw = targetYawRate - gzDps;
      iTermYaw = constrain(iTermYaw + RATE_KI_YAW * errYaw * dt, -PID_I_LIMIT_YAW, PID_I_LIMIT_YAW);
      float dYaw = RATE_KD_YAW * (errYaw - prevRateErrYaw) / dt;
      prevRateErrYaw = errYaw;
      float pidYaw = constrain(RATE_KP_YAW * errYaw + iTermYaw + dYaw, -PID_OUT_LIMIT, PID_OUT_LIMIT);

      // Limit base throttle headroom so PID authority is preserved near full stick
      int mixThrottle = constrain(baseThrottle, (int)THROTTLE_IDLE, 1850);

      // Quad-X Mixing:
      // M1 = GPIO 25 (Front-Right, CCW) : Throttle - Pitch - Roll + Yaw
      // M2 = GPIO 26 (Rear-Right,  CW)  : Throttle + Pitch - Roll - Yaw
      // M3 = GPIO 27 (Rear-Left,   CCW) : Throttle + Pitch + Roll + Yaw
      // M4 = GPIO 14 (Front-Left,  CW)  : Throttle - Pitch + Roll - Yaw
      outM1 = constrain((int)lroundf(mixThrottle - pidPitch - pidRoll + pidYaw), (int)THROTTLE_IDLE, (int)THROTTLE_MAX);
      outM2 = constrain((int)lroundf(mixThrottle + pidPitch - pidRoll - pidYaw), (int)THROTTLE_IDLE, (int)THROTTLE_MAX);
      outM3 = constrain((int)lroundf(mixThrottle + pidPitch + pidRoll + pidYaw), (int)THROTTLE_IDLE, (int)THROTTLE_MAX);
      outM4 = constrain((int)lroundf(mixThrottle - pidPitch + pidRoll - pidYaw), (int)THROTTLE_IDLE, (int)THROTTLE_MAX);
    } else {
      // Armed at minimum stick: keep motors at 1000us and prevent I-term windup
      resetPID();
    }
  }

  writeMotors(outM1, outM2, outM3, outM4);

  // -----------------------------------------------------------------------
  // Low-Rate (10 Hz) Non-Blocking Serial Telemetry
  // -----------------------------------------------------------------------
  if ((nowMs - lastTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMs;
    const char *st = (flightState == STATE_ARMED)  ? "ARMED " :
                     (flightState == STATE_KILLED) ? "KILLED" : "DISARM";
    Serial.printf("[%s] R:%+6.1f P:%+6.1f | Thr:%4d | M1:%4d M2:%4d M3:%4d M4:%4d\n",
                  st, rollAngle, pitchAngle, cmd.throttle, m1Pwm, m2Pwm, m3Pwm, m4Pwm);
  }
}
