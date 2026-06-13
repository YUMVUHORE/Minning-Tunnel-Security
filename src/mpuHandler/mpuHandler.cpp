#include "mpuHandler.h"
#include "../global.h"
#include <Wire.h>
#include <Arduino.h>

static const uint8_t MPU_addr = 0x68;
// Use zero offsets for actual sensor readings. Calibrate by setting these when
// the crutch is at rest in a known position (e.g. vertical) if you need a zero reference.
static int16_t accel_offset[3] = {0, 0, 0};
static int16_t accel_x_raw, accel_y_raw, accel_z_raw;
static float accel_x_g, accel_y_g, accel_z_g;
static float pitch, roll, angle_from_vertical;

static void calculateAngles() {
  float magYZ = accel_y_g * accel_y_g + accel_z_g * accel_z_g;
  float magXZ = accel_x_g * accel_x_g + accel_z_g * accel_z_g;
  float magnitude = sqrt(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);

  pitch = (magYZ > 1e-6f) ? (atan2(accel_x_g, sqrt(magYZ)) * 180.0f / PI) : 0.0f;
  roll  = (magXZ > 1e-6f) ? (atan2(accel_y_g, sqrt(magXZ)) * 180.0f / PI) : 0.0f;

  if (magnitude > 0.01f) {
    float cosAngle = accel_z_g / magnitude;
    if (cosAngle > 1.0f) cosAngle = 1.0f;
    if (cosAngle < -1.0f) cosAngle = -1.0f;
    angle_from_vertical = acos(cosAngle) * 180.0f / PI;
  } else {
    angle_from_vertical = 0.0f;
  }

  sensorData.pitch = pitch;
  sensorData.roll = roll;
  sensorData.angleFromVertical = angle_from_vertical;
}

// Real-time orientation from accelerometer; thresholds tuned for crutch positions
static void determineOrientation() {
  if (angle_from_vertical <= 18.0f) {
    orientation = "VERTICAL (Standing upright)";
  } else if (angle_from_vertical >= 72.0f && angle_from_vertical <= 108.0f) {
    orientation = "HORIZONTAL (Lying flat)";
  } else if (angle_from_vertical > 18.0f && angle_from_vertical <= 45.0f) {
    orientation = "SLIGHTLY INCLINED";
  } else if (angle_from_vertical > 45.0f && angle_from_vertical <= 72.0f) {
    orientation = "HIGHLY INCLINED";
  } else {
    orientation = "INVERTED";
  }
  if (abs(pitch) > 12.0f && abs(roll) < 12.0f) {
    orientation += (pitch > 0) ? " - Tilted FORWARD" : " - Tilted BACKWARD";
  } else if (abs(roll) > 12.0f && abs(pitch) < 12.0f) {
    orientation += (roll > 0) ? " - Tilted RIGHT" : " - Tilted LEFT";
  } else if (abs(pitch) > 12.0f && abs(roll) > 12.0f) {
    orientation += " - DIAGONAL tilt";
  }
  sensorData.orientation = orientation;
}

void setupMPU6050() {
  sensorData.mpuInitProgress = 0;
  Serial.println("Initializing MPU-6050...");

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) {
    sensorData.mpuInitProgress = 10;
    sensorData.mpuValid = false;
    Serial.println("MPU-6050 not found - I2C communication failed");
    return;
  }
  sensorData.mpuInitProgress = 20;
  delay(50);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) {
    sensorData.mpuInitProgress = 30;
    sensorData.mpuValid = false;
    return;
  }
  sensorData.mpuInitProgress = 40;
  delay(50);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);
  Wire.write(0x05);
  if (Wire.endTransmission(true) != 0) {
    sensorData.mpuInitProgress = 50;
    sensorData.mpuValid = false;
    return;
  }
  sensorData.mpuInitProgress = 60;
  delay(50);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  size_t bytesRead = Wire.requestFrom((uint8_t)MPU_addr, (size_t)6, (bool)true);
  if (bytesRead < 6) {
    sensorData.mpuInitProgress = 70;
    sensorData.mpuValid = false;
    Serial.println("MPU-6050 reading test failed");
    return;
  }
  sensorData.mpuInitProgress = 80;

  for (int i = 0; i < 3; i++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_addr, (size_t)6, (bool)true);
    if (Wire.available() >= 6) {
      Wire.read(); Wire.read(); Wire.read(); Wire.read(); Wire.read(); Wire.read();
    }
    delay(10);
    sensorData.mpuInitProgress = 80 + (i + 1) * 7;
  }

  sensorData.mpuValid = true;
  sensorData.mpuInitProgress = 100;
  Serial.println("MPU-6050 initialized successfully (100%)");
  delay(50);
}

void readMPU6050() {
  if (!sensorData.mpuValid) return;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
 if (Wire.requestFrom((uint8_t)MPU_addr, (uint8_t)6, (uint8_t)true) != 6)
{
    return;
}

if (Wire.available() < 6)
{
    return;
}

  accel_x_raw = Wire.read() << 8 | Wire.read();
  accel_y_raw = Wire.read() << 8 | Wire.read();
  accel_z_raw = Wire.read() << 8 | Wire.read();

  int16_t accel_x = accel_x_raw - accel_offset[0];
  int16_t accel_y = accel_y_raw - accel_offset[1];
  int16_t accel_z = accel_z_raw - accel_offset[2];

  accel_x_g = accel_x / 16384.0f;
  accel_y_g = accel_y / 16384.0f;
  accel_z_g = accel_z / 16384.0f;
static float fx = 0;
static float fy = 0;
static float fz = 0;

const float alpha = 0.2f;

fx = alpha * accel_x_g + (1.0f - alpha) * fx;
fy = alpha * accel_y_g + (1.0f - alpha) * fy;
fz = alpha * accel_z_g + (1.0f - alpha) * fz;

accel_x_g = fx;
accel_y_g = fy;
accel_z_g = fz;
  sensorData.posX = accel_x_g;
  sensorData.posY = accel_y_g;
  sensorData.posZ = accel_z_g;

  calculateAngles();
  determineOrientation();
}
