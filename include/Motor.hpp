#include <Arduino.h>
#include <ServoSmooth.h>
#include <Wire.h>

#define MIN_PWM 1000u
#define MAX_PWM 2000u

class Motor {
private:
  uint8_t _pin = 3u;
  ServoSmooth _servo;

public:
  uint16_t pwm = MIN_PWM;
  uint16_t max_throttle = MAX_PWM;

  Motor() {}
  ~Motor() {}

  void begin(uint8_t pin, uint16_t max_throttle) {
    _pin = pin;
    max_throttle = max_throttle;
    _servo.attach(_pin, MIN_PWM, MAX_PWM);
    _servo.setAutoDetach(false);
    _servo.start();
  }

  void calibrate() {
    _servo.writeMicroseconds(MAX_PWM);
    _servo.tickManual();
    delay(4000);
    _servo.writeMicroseconds(MIN_PWM);
    _servo.tickManual();
    delay(250);
  }

  void go(uint16_t pwm) {
    constrain(pwm, MIN_PWM, MAX_PWM);
    this->pwm = pwm;
    _servo.writeMicroseconds(pwm);
  }

  void stop() {
    pwm = MIN_PWM;
    _servo.writeMicroseconds(pwm);
  }
};