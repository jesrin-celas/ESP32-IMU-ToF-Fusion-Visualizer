#ifndef MYVL53_H
#define MYVL53_H

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

class MyVL53 {
  public:
    bool begin();
    uint16_t readDistance();

  private:
    Adafruit_VL53L0X lox;

    static const int N = 5;
    uint16_t buffer[N];
    int index = 0;
    float alpha = 0.6;      // exponential smoothing factor
    float filtered = 0;

    uint16_t medianFilter(uint16_t val);
};

#endif
