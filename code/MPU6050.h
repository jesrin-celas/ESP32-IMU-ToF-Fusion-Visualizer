#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
private:
  const uint8_t MPU_ADDRESS = 0x68;
  
  // Raw sensor data
  int16_t accXRaw, accYRaw, accZRaw;
  int16_t gyroXRaw, gyroYRaw, gyroZRaw;
  
  // Calibration offsets
  float gyroRollOffset, gyroPitchOffset, gyroYawOffset;
  float accelRollOffset, accelPitchOffset;
  
public:
  // Processed sensor data
  float accX, accY, accZ;
  float gyroRoll, gyroPitch, gyroYaw;
  float accelRoll, accelPitch;
  
  MPU6050();
  
  void begin(uint8_t sdaPin, uint8_t sclPin);
  void readSensors();
  void calibrateGyro(int samples = 500);
  void calibrateAccel(int samples = 100);
  
  float getGyroRoll() { return gyroRoll - gyroRollOffset; }
  float getGyroPitch() { return gyroPitch - gyroPitchOffset; }
  float getGyroYaw() { return gyroYaw - gyroYawOffset; }
  
  float getCorrectedAccelRoll() { return accelRoll - accelRollOffset; }
  float getCorrectedAccelPitch() { return accelPitch - accelPitchOffset; }
};

#endif