#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <Arduino.h>

class ComplementaryFilter {
private:
  float alpha;  // Filter coefficient (0-1)
  unsigned long previousTime;
  
public:
  float roll, pitch;
  
  ComplementaryFilter(float filterCoeff = 0.96);
  
  void begin();
  void update(float gyroRoll, float gyroPitch, float accelRoll, float accelPitch);
  void reset();
  
  float getRoll() { return constrain(roll, -90, 90); }
  float getPitch() { return constrain(pitch, -90, 90); }
};

#endif