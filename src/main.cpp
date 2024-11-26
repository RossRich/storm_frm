#include <Arduino.h>
#include <GyverHX711.h>

GyverHX711 weight_sensor(3, 2, HX_GAIN64_A);

void setup() { 
  Serial.begin(115200);
  delay(500);
  weight_sensor.tare();
}

void loop() {
  if (weight_sensor.available())
    Serial.println(weight_sensor.read());
}