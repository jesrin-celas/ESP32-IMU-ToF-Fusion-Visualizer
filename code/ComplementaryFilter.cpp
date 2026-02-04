/***********************************************************************
 *  Project: ESP32 IMU + ToF Fusion Visualizer
 *  File: ComplementaryFilter.cpp
 *
 *  Copyright (c) 2026 Jesrin Celas
 *
 *  Author: Jesrin Celas A S
 *  Description:
 *  Complementary filter for roll and pitch estimation. Combines
 *  gyroscope integration with accelerometer absolute angle reference.
 ***********************************************************************/

#include "ComplementaryFilter.h"
#include <math.h>

ComplementaryFilter::ComplementaryFilter(float filterCoeff) {
  alpha = filterCoeff;
  roll = 0.0f;
  pitch = 0.0f;
  previousTime = 0;
}

void ComplementaryFilter::begin() {
  roll = 0.0f;
  pitch = 0.0f;
  previousTime = micros();
}

void ComplementaryFilter::update(float gyroRoll, float gyroPitch,
                                 float accelRoll, float accelPitch) {

  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) * 1e-6f;
  previousTime = currentTime;

  // Reject bad dt values (timing glitch protection)
  if (dt < 0.002f || dt > 0.05f) return;

  // -------- Gyro integration --------
  float gyroRollPred = roll + gyroRoll * dt;
  float gyroPitchPred = pitch + gyroPitch * dt;

  // -------- Accel sanity check --------
  // Reject extreme spikes (>45° jump)
  if (fabs(accelRoll - roll) > 45.0f) accelRoll = roll;
  if (fabs(accelPitch - pitch) > 45.0f) accelPitch = pitch;

  // -------- Complementary fusion --------
  roll  = alpha * gyroRollPred  + (1.0f - alpha) * accelRoll;
  pitch = alpha * gyroPitchPred + (1.0f - alpha) * accelPitch;
}

void ComplementaryFilter::reset() {
  roll = 0.0f;
  pitch = 0.0f;
  previousTime = micros();
}

float ComplementaryFilter::getRoll() {
  return roll;
}

float ComplementaryFilter::getPitch() {
  return pitch;
}
