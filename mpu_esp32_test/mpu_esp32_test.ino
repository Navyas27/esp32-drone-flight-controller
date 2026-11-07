/*
 * Device 0x53 Identification & Telemetry
 */

#include <Wire.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define SENSOR_ADDR 0x53

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n=============================================="));
  Serial.println(F("       Probing I2C Device at Address 0x53     "));
  Serial.println(F("=============================================="));

  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Wire.setTimeOut(100);

  // Read register 0x00 (Device ID)
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)1);

  if (Wire.available()) {
    uint8_t devId = Wire.read();
    Serial.print(F("Device ID Register (0x00) returned: 0x"));
    Serial.println(devId, HEX);

    if (devId == 0xE5) {
      Serial.println(F("\n>>> CONFIRMED: This sensor is an ADXL345 Accelerometer! <<<"));

      // Put ADXL345 into measurement mode (write 0x08 to register 0x2D POWER_CTL)
      Wire.beginTransmission(SENSOR_ADDR);
      Wire.write(0x2D);
      Wire.write(0x08);
      Wire.endTransmission();

      // Set range to +-4g (write 0x01 to 0x31 DATA_FORMAT)
      Wire.beginTransmission(SENSOR_ADDR);
      Wire.write(0x31);
      Wire.write(0x01);
      Wire.endTransmission();

      Serial.println(F("ADXL345 Initialized in Measurement Mode (±4g).\n"));
      Serial.println(F("Streaming Live 3-Axis Acceleration:"));
      Serial.println(F("   X-Axis\t   Y-Axis\t   Z-Axis\t Total (g)"));
      Serial.println(F("------------\t------------\t------------\t----------"));
    } else {
      Serial.print(F("Unknown chip ID 0x"));
      Serial.println(devId, HEX);
    }
  } else {
    Serial.println(F("Failed to read register 0x00 from 0x53."));
  }
}

void loop() {
  // Read 6 bytes starting at 0x32 (DATAX0) for ADXL345
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x32);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)6) == 6) {
    int16_t x = Wire.read() | (Wire.read() << 8);
    int16_t y = Wire.read() | (Wire.read() << 8);
    int16_t z = Wire.read() | (Wire.read() << 8);

    // Scale for +-4g (7.8 mg/LSB or / 128.0)
    float ax = (float)x * 0.0078;
    float ay = (float)y * 0.0078;
    float az = (float)z * 0.0078;
    float total = sqrt(ax * ax + ay * ay + az * az);

    Serial.print(ax, 3); Serial.print(F(" g\t"));
    Serial.print(ay, 3); Serial.print(F(" g\t"));
    Serial.print(az, 3); Serial.print(F(" g\t"));
    Serial.print(total, 3); Serial.println(F(" g"));
  }
  delay(200);
}
