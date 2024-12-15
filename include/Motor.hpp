#include <Arduino.h>
#include <ServoSmooth.h>
#include <TimerMs.h>
#include <Wire.h>

#define MIN_PWM 1000u
#define MAX_PWM 1950u

class Motor {
private:
  uint8_t _pin = 3u;
  ServoSmooth _servo;

public:
  uint16_t pwm = MIN_PWM;
  uint16_t max_throttle = MAX_PWM;
  volatile bool is_calibration_done = false;

  Motor() {}
  ~Motor() {}

  void begin(uint8_t pin, uint16_t max_throttle) {
    _pin = pin;
    this->max_throttle = max_throttle;
    is_calibration_done = false;
    _servo.attach(_pin, MIN_PWM, MAX_PWM);
    _servo.setAutoDetach(false);
    _servo.start();
  }

  void calibrate() {
    _servo.writeMicroseconds(MAX_PWM);
    _servo.tickManual();
    delay(6000);
    _servo.writeMicroseconds(MIN_PWM);
    _servo.tickManual();
    delay(3000);
    is_calibration_done = true;
  }

  void go(uint16_t pwm) {
    if (not is_calibration_done)
      return;

    pwm = constrain(pwm, MIN_PWM, MIN_PWM + max_throttle);
    this->pwm = pwm;
    _servo.writeMicroseconds(this->pwm);
    _servo.tickManual();
  }

  void stop() {
    uint16_t _pwm = int(max_throttle / 10.0f);
    for (size_t i = 0; i < 10; i++)
    {
      pwm = pwm - _pwm;
      pwm = constrain(pwm, MIN_PWM, MAX_PWM);
      _servo.writeMicroseconds(pwm);
      _servo.tickManual();
      delay(100);
    }
  }
};