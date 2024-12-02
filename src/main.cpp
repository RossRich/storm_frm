#include <ACS758_50B.hpp>
#include <Arduino.h>
#include <GyverFilters.h>
#include <GyverHX711.h>

#define HX_DT_PIN 8           //< Тензо-датчик
#define HX_SCK_PIN 7          //< Тензо-датчик
#define CURRENT_SENSOR_PIN A0 //< ACS758
#define VCC_CUSTOM_PIN 4

#define SERIAL_MOODE 0
#define WEB_MOODE 1

GyverHX711 weight_sensor(HX_DT_PIN, HX_SCK_PIN, HX_GAIN64_A);
ACS758_50B current_sensor;

uint8_t mode = SERIAL_MOODE;

void setup() {
  Serial.begin(115200);
  pinMode(VCC_CUSTOM_PIN, OUTPUT);
  digitalWrite(VCC_CUSTOM_PIN, HIGH);
  current_sensor.begin(CURRENT_SENSOR_PIN);
  weight_sensor.sleepMode(false);
  weight_sensor.tare();
  delay(500);
  weight_sensor.read();
}

void data_to_serail(const long &weight, const float &current,
                    const float &voltage) {
  Serial.print(weight);
  Serial.print(" ");
  Serial.print(current);
  Serial.print(" ");
  Serial.println(voltage);
}

// void data_to_web()

void loop() {
  static long weight = 0;
  static float current = 0;

  if (weight_sensor.available()) {
    weight = weight_sensor.read();
  }

  current = current_sensor.current();

  data_to_serail(weight, current, current_sensor.adc_voltage());

  delay(100);
}