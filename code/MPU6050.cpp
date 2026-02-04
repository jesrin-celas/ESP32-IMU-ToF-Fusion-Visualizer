/***********************************************************************
 *  Project: ESP32 IMU + ToF Fusion Visualizer
 *  File: MPU6050.cpp
 *
 *  Copyright (c) 2026 Jesrin Celas
 *
 *  Author: Jesrin Celas A S
 *  Description:
 *  Low-level MPU6050 driver with gyro & accelerometer calibration,
 *  angle computation, and sensor conversion for fusion algorithms.
 ***********************************************************************/

#include "MPU6050.h"
#include <math.h>

MPU6050::MPU6050() {
  gyroRollOffset = gyroPitchOffset = gyroYawOffset = 0.0f;
  accelRollOffset = accelPitchOffset = 0.0f;
}

void MPU6050::begin(uint8_t sdaPin, uint8_t sclPin) {
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(400000);
  delay(300);

  writeRegister(0x6B, 0x00);  // Wake
  writeRegister(0x1A, 0x05);  // DLPF
  writeRegister(0x1B, 0x08);  // Gyro ±500°/s
  writeRegister(0x1C, 0x10);  // Accel ±8g
}

void MPU6050::writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void MPU6050::readSensors() {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;

  Wire.requestFrom(MPU_ADDRESS, (uint8_t)14);
  if (Wire.available() < 14) return;

  accXRaw = (Wire.read() << 8) | Wire.read();
  accYRaw = (Wire.read() << 8) | Wire.read();
  accZRaw = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // temp
  gyroXRaw = (Wire.read() << 8) | Wire.read();
  gyroYRaw = (Wire.read() << 8) | Wire.read();
  gyroZRaw = (Wire.read() << 8) | Wire.read();

  // ---- Convert units ----
  accX = accXRaw * (1.0f / 4096.0f);
  accY = accYRaw * (1.0f / 4096.0f);
  accZ = accZRaw * (1.0f / 4096.0f);

  gyroRoll  = (gyroXRaw * (1.0f / 65.5f)) - gyroRollOffset;
  gyroPitch = (gyroYRaw * (1.0f / 65.5f)) - gyroPitchOffset;
  gyroYaw   = (gyroZRaw * (1.0f / 65.5f)) - gyroYawOffset;

  accelRoll  = atan2f(accY, sqrtf(accX*accX + accZ*accZ)) * RAD_TO_DEG - accelRollOffset;
  accelPitch = atan2f(-accX, sqrtf(accY*accY + accZ*accZ)) * RAD_TO_DEG - accelPitchOffset;
}

void MPU6050::calibrateGyro(int samples) {

  gyroRollOffset = gyroPitchOffset = gyroYawOffset = 0.0f;

  for (int i = 0; i < samples; i++) {
    readSensors();
    gyroRollOffset  += gyroXRaw * (1.0f / 65.5f);
    gyroPitchOffset += gyroYRaw * (1.0f / 65.5f);
    gyroYawOffset   += gyroZRaw * (1.0f / 65.5f);
    delay(5);
  }

  gyroRollOffset  /= samples;
  gyroPitchOffset /= samples;
  gyroYawOffset   /= samples;
}

void MPU6050::calibrateAccel(int samples) {

  accelRollOffset = accelPitchOffset = 0.0f;

  for (int i = 0; i < samples; i++) {
    readSensors();
    accelRollOffset  += accelRoll;
    accelPitchOffset += accelPitch;
    delay(10);
  }

  accelRollOffset  /= samples;
  accelPitchOffset /= samples;
}
