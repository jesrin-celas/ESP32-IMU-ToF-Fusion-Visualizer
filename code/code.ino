/***********************************************************************
 *  Project: ESP32 IMU + ToF Fusion Visualizer
 *  File: main.ino
 *
 *  Copyright (c) 2026 Jesrin Celas
 *
 *  Author: Jesrin Celas A S
 *  Description:
 *  Real-time roll, pitch and tilt-compensated height estimation using
 *  MPU6050 IMU and VL53L0X ToF sensor. Data streamed via UART for 3D
 *  visualization.
 ***********************************************************************/

#include <math.h>
#include "MPU6050.h"
#include "ComplementaryFilter.h"
#include "MyVL53.h"

#define BAUD_RATE 115200
#define LOOP_HZ 50
#define LOOP_PERIOD_MS (1000 / LOOP_HZ)

const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;
const uint8_t LED_PIN = 13;

MPU6050 imu;
ComplementaryFilter filter(0.96);
MyVL53 tof;

float lastHeight = 0.0f;
unsigned long lastLoopTime = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println("IMU + ToF Fusion System");
  Serial.println("Format: roll,pitch,height(m)");

  imu.begin(SDA_PIN, SCL_PIN);
  imu.calibrateGyro(500);
  imu.calibrateAccel(100);

  filter.begin();

  if (!tof.begin()) {
    Serial.println("ToF init failed");
    while (1);
  }

  Serial.println("System Ready");
}

void loop() {

  // -------- Fixed Loop Timing --------
  if (millis() - lastLoopTime < LOOP_PERIOD_MS) return;
  lastLoopTime = millis();

  // -------- IMU --------
  imu.readSensors();

  filter.update(
    imu.getGyroRoll(),
    imu.getGyroPitch(),
    imu.getCorrectedAccelRoll(),
    imu.getCorrectedAccelPitch()
  );

  float roll_deg = filter.getRoll();
  float pitch_deg = filter.getPitch();

  float roll = roll_deg * DEG_TO_RAD;
  float pitch = pitch_deg * DEG_TO_RAD;

  // -------- ToF --------
  uint16_t D_mm = tof.readDistance();
  float height = lastHeight;

  if (D_mm > 0 && D_mm < 2000) {
    float D = D_mm * 0.001f;   // mm → meters
    height = D * cosf(roll) * cosf(pitch);
    lastHeight = height;
  }

  // -------- UART --------
  Serial.print(roll_deg, 2);
  Serial.print(",");
  Serial.print(pitch_deg, 2);
  Serial.print(",");
  Serial.println(height, 3);
}
