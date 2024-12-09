#include "Config.hpp"
#include <Arduino.h>

#define SCALE 0.09856f

class Volt {
private:
  uint8_t _pin = A0;

public:
  Volt() {}
  ~Volt() {}

  bool begin(uint8_t pin, uint16_t timeout_ms = 5000u) {
    _pin = pin;
    pinMode(_pin, INPUT);
  }

  inline float value() {
    return (analogRead(A1) * VCC_REF / static_cast<float>(ADC_MAX)) / SCALE;
  }
};