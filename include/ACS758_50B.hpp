#include "Config.hpp"
#include <Arduino.h>

#define ACS758_50B_SENS 40.0f //< Коффициент mV/A для расчета силы тока
#define ACS758_50B_QOV 0.5f //< Коэффициент VCC при котором 0 ампер

// Расчет силы тока
// ADC_VOLTAGE_FACTOR = VCC_REF / ADC_MAX
// SENSOR_OFFSET_FACTOR = VCC_REF * ACS758_50B_QOV
// SENSOR_FACTOR = ACS758_50B_SENS / 1000
// I = (SENSOR_OFFSET_FACTOR - ADC * ADC_VOLTAGE_FACTOR) / SENSOR_FACTOR

class ACS758_50B {
private:
  uint8_t _data_pin = A0;
  const float _sensor_factor = ACS758_50B_SENS / 1000.0f;
  const float _offset_factor = VCC_REF * ACS758_50B_QOV;
  int _offset_adc = 0;

public:
  ACS758_50B() {}
  ~ACS758_50B() {}

  bool begin(const uint8_t sensor_pin, uint16_t timeout_ms = 5000u) {
    _data_pin = sensor_pin;
    pinMode(_data_pin, INPUT);

    uint64_t t = millis() + timeout_ms;
    while (not analogRead(_data_pin)) {
      if (millis() > t) {
        break;
      } else {
        delay(100);
      }
    }

    float offset = 0.0f;
    uint8_t samples = 16;
    for (size_t i = 0; i < samples; i++) {
      offset += analogRead(_data_pin);
      delay(25);
    }

    offset /= samples;
    _offset_adc = static_cast<uint16_t>(offset);

    if (_offset_adc == 0)
      return false;

    return true;
  }

  inline float current() {
    return (static_cast<float>(analogRead(_data_pin) - _offset_adc) *
            ADC_CONVERT_FACTOR) /
           _sensor_factor;
  }

  inline float adc_voltage() {
    return analogRead(_data_pin) * ADC_CONVERT_FACTOR;
  }
};