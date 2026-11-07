/*
 * MPU6050 I2C Test Sketch (Robust)
 * ---------------------------------
 * Wiring:
 *   SDA -> A4  (Arduino Uno)
 *   SCL -> A5  (Arduino Uno)
 *   VCC -> 3.3V or 5V
 *   GND -> GND
 *
 * Opens Serial at 115200 baud and prints raw accel/gyro values.
 */

#include <Wire.h>

#define MPU6050_ADDR 0x68

// Register addresses
#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT 0x3B  // Start of 14-byte burst (accel, temp, gyro)

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t temperature;

void i2cScan() {
  Serial.println(F("\n--- I2C Bus Scan ---"));
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(F("  Device found at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.println(F("  No I2C devices found!"));
  } else {
    Serial.print(F("  Total: "));
    Serial.print(count);
    Serial.println(F(" device(s)"));
  }
  Serial.println(F("--------------------\n"));
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial + MPU to power up

  Wire.begin();  // SDA=A4, SCL=A5 (hardware default on Uno)

  Serial.println();
  Serial.println(F("==========================="));
  Serial.println(F(" MPU6050 I2C Test"));
  Serial.println(F(" SDA=A4  SCL=A5"));
  Serial.println(F("==========================="));

  // --- Scan the I2C bus first ---
  i2cScan();

  // --- Try to contact MPU6050 ---
  Wire.beginTransmission(MPU6050_ADDR);
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print(F("[ERROR] MPU6050 not responding at 0x68. I2C error code: "));
    Serial.println(error);
    Serial.println(F("Check wiring:"));
    Serial.println(F("  SDA -> A4,  SCL -> A5"));
    Serial.println(F("  VCC -> 3.3V/5V,  GND -> GND"));
    Serial.println(F("Halting. Press reset to retry."));
    while (1) { delay(1000); }
  }

  // --- Check WHO_AM_I register (should return 0x68) ---
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);

  if (Wire.available()) {
    uint8_t whoAmI = Wire.read();
    Serial.print(F("WHO_AM_I = 0x"));
    Serial.println(whoAmI, HEX);

    if (whoAmI == 0x68 || whoAmI == 0x98) {
      Serial.println(F("[OK] MPU6050 detected!"));
    } else {
      Serial.println(F("[WARN] Unexpected WHO_AM_I value — may be a clone."));
    }
  }

  // --- Wake up MPU6050 (clear sleep bit) ---
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);  // Clear sleep bit
  Wire.endTransmission();
  delay(100);

  Serial.println(F("\nMPU6050 initialized. Streaming data...\n"));
  Serial.println(F("  AX\t  AY\t  AZ\t  GX\t  GY\t  GZ\tTemp(C)"));
  Serial.println(F("------\t------\t------\t------\t------\t------\t-------"));
}

void loop() {
  // --- Burst-read 14 bytes starting at 0x3B ---
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  if (Wire.available() < 14) {
    Serial.println(F("[ERROR] Read failed — check wiring"));
    delay(1000);
    return;
  }

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  temperature = (Wire.read() << 8) | Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  float tempC = temperature / 340.0 + 36.53;

  Serial.print(ax); Serial.print('\t');
  Serial.print(ay); Serial.print('\t');
  Serial.print(az); Serial.print('\t');
  Serial.print(gx); Serial.print('\t');
  Serial.print(gy); Serial.print('\t');
  Serial.print(gz); Serial.print('\t');
  Serial.println(tempC, 1);

  delay(500);
}
