/***********************************************************************
 *  Project: ESP32 IMU + ToF Fusion Visualizer
 *  File: MyVL53.cpp
 *
 *  Copyright (c) 2026 Jesrin Celas
 *
 *  Author: Jesrin Celas A S
 *  Description:
 *  VL53L0X wrapper with median + exponential filtering for stable
 *  distance measurements in dynamic environments.
 ***********************************************************************/

#include "MyVL53.h"
#include <string.h>

bool MyVL53::begin() {

  if (!lox.begin()) return false;

  for (int i = 0; i < N; i++) buffer[i] = 0;

  index = 0;
  filtered = 0.0f;

  return true;
}

uint16_t MyVL53::medianFilter(uint16_t val) {

  buffer[index++] = val;
  if (index >= N) index = 0;

  uint16_t temp[N];
  memcpy(temp, buffer, sizeof(temp));

  for (int i = 0; i < N - 1; i++)
    for (int j = i + 1; j < N; j++)
      if (temp[j] < temp[i]) {
        uint16_t t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }

  return temp[N / 2];
}

uint16_t MyVL53::readDistance() {

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  // -------- Reject invalid measurement --------
  if (measure.RangeStatus == 4) {
    return (uint16_t)filtered;
  }

  uint16_t raw = measure.RangeMilliMeter;

  // -------- Physical bounds check (sensor spec) --------
  if (raw < 30 || raw > 2000) {
    return (uint16_t)filtered;
  }

  // -------- Median filter (spike removal) --------
  uint16_t med = medianFilter(raw);

  // -------- Exponential smoothing --------
  filtered = alpha * med + (1.0f - alpha) * filtered;

  return (uint16_t)filtered;
}
