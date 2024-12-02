#include <Arduino.h>

#define ADC_MAX 1024u //< Максимальное значение АЦП
#define VCC_REF 4.98f //< Напряжение питание платы
#define ACS758_50B_SENS 40.0f //< Коффициент mV/A для расчета силы тока
#define ACS758_50B_QOV 0.5f //< Коэффициент VCC при котором 0 ампер

// Расчет силы тока
// ADC_VOLTAGE_FACTOR = VCC_REF / ADC_MAX
// SENSOR_OFFSET_FACTOR = VCC_REF * ACS758_50B_QOV
// SENSOR_FACTOR = ACS758_50B_SENS / 1000
// I = (SENSOR_OFFSET_FACTOR - ADC * ADC_VOLTAGE_FACTOR) * SENSOR_FACTOR

class ACS758_50B {
private:
  uint8_t _data_pin = A0;
  const float _sensor_factor = ACS758_50B_SENS / 1000.0f;
  const float _current_factor = VCC_REF / float(ADC_MAX);
  const float _offset_factor = VCC_REF * ACS758_50B_QOV;

public:
  ACS758_50B() {}
  ~ACS758_50B() {}

  void begin(const uint8_t sensor_pin) {
    _data_pin = sensor_pin;
    pinMode(_data_pin, INPUT);
  }

  inline float current() {
    return (_offset_factor - analogRead(_data_pin) * _current_factor) /
           _sensor_factor;
  }

  inline float adc_voltage() { return analogRead(_data_pin) * _current_factor; }
};