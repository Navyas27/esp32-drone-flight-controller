/*
 * =========================================================================
 * ESP32 Ultrasonic Rangefinder Calibration & Diagnostic Utility
 * =========================================================================
 * Target       : ESP32 Dev Module
 * Sensors      : HC-SR04 / US-100 / RCWL-1601 & optional MPU6050 (Temp comp)
 * Safe Pins    : TRIG -> GPIO 18, ECHO -> GPIO 19
 *
 * SAFETY WARNING FOR 5V HC-SR04:
 * The HC-SR04 Echo pin outputs 5V logic! Connect Echo to ESP32 GPIO 19
 * through a simple resistor voltage divider:
 *   ECHO (5V) --- [1k to 2.2k ohm] ---+--- GPIO 19 (3.3V safe)
 *                                      |
 *                                [2k to 3.3k ohm]
 *                                      |
 *                                     GND
 * (Alternatively, use 3.3V-tolerant sensors like HC-SR04P, US-100, RCWL-1601)
 * =========================================================================
 */

#include <Arduino.h>
#include <Wire.h>

// --- Pin Definitions ---
const int PIN_TRIG = 18;
const int PIN_ECHO = 19;

// I2C Pins for MPU6050 (optional temperature compensation)
const int PIN_SDA  = 21;
const int PIN_SCL  = 22;
const uint8_t MPU_ADDR = 0x68;

// --- Calibration Variables ---
float scaleMultiplier = 1.0f;       // Calibration slope adjustment
float groundOffsetCm = 0.0f;        // Sensor height when drone rests on ground (skids touching)
float defaultTempC = 25.0f;         // Fallback ambient temperature
bool mpuAvailable = false;

// Median filter buffer for clean calibration readings
const int NUM_SAMPLES = 25;
float samples[NUM_SAMPLES];

// --- Non-blocking / Safe Echo Timing ---
// Reads ultrasonic pulse duration in microseconds (timeout 25ms ~ 4.2m)
long measureEchoMicroseconds() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 25000);
  return duration;
}

// Read temperature from MPU6050 if available
float getAmbientTemperature() {
  if (!mpuAvailable) return defaultTempC;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x41); // TEMP_OUT_H register
  if (Wire.endTransmission(false) != 0) return defaultTempC;

  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true);
  if (Wire.available() >= 2) {
    int16_t rawTemp = (Wire.read() << 8) | Wire.read();
    return (rawTemp / 340.0f) + 36.53f; // Temperature in deg C
  }
  return defaultTempC;
}

// Calculate sound speed in cm per microsecond compensated for temperature
// Speed of sound in dry air: v ≈ 331.3 + 0.606 * Temp(°C) [m/s]
float getSoundSpeedCmPerUs(float tempC) {
  float v_mps = 331.3f + (0.606f * tempC);
  return (v_mps * 100.0f) / 1000000.0f; // cm / microsecond
}

// Convert raw echo microseconds to raw distance (cm)
float echoToRawDistanceCm(long echoUs, float tempC) {
  if (echoUs <= 0) return -1.0f;
  float speed = getSoundSpeedCmPerUs(tempC);
  return (echoUs / 2.0f) * speed;
}

// Collect N samples, filter out timeouts, and compute median
float getFilteredReadingCm(int count = 15) {
  if (count > NUM_SAMPLES) count = NUM_SAMPLES;
  float validReadings[NUM_SAMPLES];
  int validCount = 0;
  float tempC = getAmbientTemperature();

  for (int i = 0; i < count; i++) {
    long echo = measureEchoMicroseconds();
    if (echo > 120 && echo < 24000) { // Valid range ~2cm to 400cm
      validReadings[validCount++] = echoToRawDistanceCm(echo, tempC);
    }
    delay(20); // 50Hz ultrasonic burst spacing to avoid secondary echo interference
  }

  if (validCount < (count / 2)) {
    return -1.0f; // Too many dropouts / invalid echo
  }

  // Sort validReadings for median extraction
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = i + 1; j < validCount; j++) {
      if (validReadings[j] < validReadings[i]) {
        float temp = validReadings[i];
        validReadings[i] = validReadings[j];
        validReadings[j] = temp;
      }
    }
  }

  return validReadings[validCount / 2];
}

// Flush serial buffer
void flushInput() {
  while (Serial.available()) Serial.read();
}

// Wait for user to press ENTER
void waitForEnter() {
  flushInput();
  while (!Serial.available()) {
    delay(10);
  }
  flushInput();
}

void printMenu() {
  Serial.println(F("\n======================================================="));
  Serial.println(F("       ESP32 ULTRASONIC SENSOR CALIBRATION MENU        "));
  Serial.println(F("======================================================="));
  Serial.println(F(" [1] Live Continuous Stream (Test echo, noise & jitter)"));
  Serial.println(F(" [2] Calibrate Zero-Offset (Landing Skid / Ground Level)"));
  Serial.println(F(" [3] 2-Point Scale Calibration (Ruler Accuracy Tuning) "));
  Serial.println(F(" [4] Print Final Calibration Config for Drone Firmware "));
  Serial.println(F("======================================================="));
  Serial.print(F("Enter choice (1-4): "));
}

// ---------------------------------------------------------
// Mode 1: Continuous Live Stream & Health Check
// ---------------------------------------------------------
void runLiveStream() {
  Serial.println(F("\n--- Mode 1: Live Stream (Press any key + Enter to return to menu) ---"));
  Serial.println(F("Raw_Echo(us)\tTemp(C)\tRaw_Dist(cm)\tCalib_Alt(cm)\tStatus"));

  while (!Serial.available()) {
    float tempC = getAmbientTemperature();
    long echoUs = measureEchoMicroseconds();
    float rawDist = echoToRawDistanceCm(echoUs, tempC);
    
    Serial.print(echoUs);
    Serial.print(F("\t\t"));
    Serial.print(tempC, 1);
    Serial.print(F("\t"));

    if (echoUs <= 0 || rawDist < 0) {
      Serial.print(F("NO ECHO\t\t---\t\t[OUT OF RANGE / BLIND SPOT]"));
    } else {
      float calibDist = (rawDist * scaleMultiplier) - groundOffsetCm;
      Serial.print(rawDist, 2);
      Serial.print(F("\t\t"));
      Serial.print(calibDist, 2);
      Serial.print(F("\t\t"));
      if (rawDist < 3.5f) {
        Serial.print(F("[BLIND ZONE < 3cm]"));
      } else if (rawDist > 300.0f) {
        Serial.print(F("[NEAR MAX RANGE]"));
      } else {
        Serial.print(F("[OK - RELIABLE]"));
      }
    }
    Serial.println();
    delay(100);
  }
  flushInput();
}

// ---------------------------------------------------------
// Mode 2: Calibrate Ground Offset (Skid Clearance)
// ---------------------------------------------------------
void runGroundZeroCalibration() {
  Serial.println(F("\n======================================================="));
  Serial.println(F("      STEP 1: GROUND ZERO-OFFSET CALIBRATION           "));
  Serial.println(F("======================================================="));
  Serial.println(F("1. Place your quadcopter on a flat, hard, level floor."));
  Serial.println(F("2. Ensure landing skids/legs are fully resting on the floor."));
  Serial.println(F("3. The downward ultrasonic sensor will measure its height above"));
  Serial.println(F("   the floor. This height will be subtracted so that:"));
  Serial.println(F("   ALTITUDE = 0.0 cm when the drone is on the ground."));
  Serial.println(F("Press [ENTER] when ready to measure..."));
  waitForEnter();

  Serial.println(F("\nSampling 30 pulses... keep drone still..."));
  float zeroReading = getFilteredReadingCm(30);

  if (zeroReading < 0) {
    Serial.println(F("[ERROR] Could not obtain stable ground echo! Sensor may be too close (< 3cm) or blocked."));
    return;
  }

  groundOffsetCm = zeroReading;
  Serial.println(F("--> Calibration Successful!"));
  Serial.print(F("Sensor Ground Clearance Offset = "));
  Serial.print(groundOffsetCm, 2);
  Serial.println(F(" cm"));
}

// ---------------------------------------------------------
// Mode 3: 2-Point Reference Scale Calibration
// ---------------------------------------------------------
void runTwoPointCalibration() {
  Serial.println(F("\n======================================================="));
  Serial.println(F("        STEP 2: 2-POINT SCALE CALIBRATION              "));
  Serial.println(F("======================================================="));
  Serial.println(F("This step tunes the scale factor to eliminate linear ruler"));
  Serial.println(F("measurement errors. Have a flat cardboard box or wall and ruler."));
  Serial.println();
  
  // Point 1
  Serial.println(F("--> POINT 1: Place a flat target exactly 20.0 cm in front/under the sensor."));
  Serial.println(F("Press [ENTER] when target is positioned at 20 cm..."));
  waitForEnter();
  Serial.println(F("Sampling Point 1..."));
  float d1 = getFilteredReadingCm(25);
  if (d1 < 0) {
    Serial.println(F("[ERROR] Failed to read Point 1. Returning to menu."));
    return;
  }
  Serial.print(F("Measured Distance 1: "));
  Serial.print(d1, 2);
  Serial.println(F(" cm (Actual: 20.0 cm)"));

  // Point 2
  Serial.println(F("\n--> POINT 2: Place the flat target exactly 60.0 cm away."));
  Serial.println(F("Press [ENTER] when target is positioned at 60 cm..."));
  waitForEnter();
  Serial.println(F("Sampling Point 2..."));
  float d2 = getFilteredReadingCm(25);
  if (d2 < 0) {
    Serial.println(F("[ERROR] Failed to read Point 2. Returning to menu."));
    return;
  }
  Serial.print(F("Measured Distance 2: "));
  Serial.print(d2, 2);
  Serial.println(F(" cm (Actual: 60.0 cm)"));

  // Compute scale multiplier: (Actual_Span) / (Measured_Span)
  float actualSpan = 60.0f - 20.0f; // 40.0 cm
  float measuredSpan = d2 - d1;

  if (abs(measuredSpan) < 5.0f) {
    Serial.println(F("[ERROR] Measured span is too small! Did the target move? Calibration aborted."));
    return;
  }

  scaleMultiplier = actualSpan / measuredSpan;
  Serial.println(F("\n--> Calibration Complete!"));
  Serial.print(F("Calculated Scale Multiplier: "));
  Serial.println(scaleMultiplier, 4);
}

// ---------------------------------------------------------
// Mode 4: Print Final Header Configuration for Drone
// ---------------------------------------------------------
void printFlightControllerConfig() {
  Serial.println(F("\n======================================================="));
  Serial.println(F("  COPY & PASTE CONFIGURATION FOR flight_controller.ino "));
  Serial.println(F("======================================================="));
  Serial.println(F("// Ultrasonic Rangefinder Configuration"));
  Serial.println(F("#define PIN_ULTRASONIC_TRIG  18"));
  Serial.println(F("#define PIN_ULTRASONIC_ECHO  19"));
  Serial.println();
  Serial.print(F("const float US_SCALE_MULTIPLIER = "));
  Serial.print(scaleMultiplier, 4);
  Serial.println(F("f;"));
  Serial.print(F("const float US_GROUND_OFFSET_CM = "));
  Serial.print(groundOffsetCm, 2);
  Serial.println(F("f; // Subtracts physical skid clearance"));
  Serial.println(F("const float US_MIN_VALID_ALT_CM = 3.0f;"));
  Serial.println(F("const float US_MAX_VALID_ALT_CM = 250.0f;"));
  Serial.println(F("=======================================================\n"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // Initialize I2C for MPU6050
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setTimeOut(50);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    mpuAvailable = true;
    Serial.println(F("[INFO] MPU6050 detected! Real-time temperature compensation enabled."));
  } else {
    Serial.println(F("[INFO] MPU6050 not detected. Using default 25 deg C sound speed."));
  }

  printMenu();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    flushInput();

    switch (cmd) {
      case '1':
        runLiveStream();
        printMenu();
        break;
      case '2':
        runGroundZeroCalibration();
        printMenu();
        break;
      case '3':
        runTwoPointCalibration();
        printMenu();
        break;
      case '4':
        printFlightControllerConfig();
        printMenu();
        break;
      default:
        Serial.println(F("Invalid selection. Choose 1, 2, 3, or 4."));
        printMenu();
        break;
    }
  }
}
