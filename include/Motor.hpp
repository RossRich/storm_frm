#include <Arduino.h>
#include <ServoSmooth.h>
#include <TimerMs.h>
#include <Wire.h>

#define TRANK_PWM 50u
#define MIN_PWM 1000u
#define MAX_PWM 2000u

class Motor {
private:
  uint8_t _pin = 3u;
  ServoSmooth _servo;

public:
  uint16_t pwm = MIN_PWM;
  uint16_t max_throttle_m = (MAX_PWM - MIN_PWM) - TRANK_PWM;
  bool is_calibration_done = false;

  Motor() {}
  ~Motor() {}

  bool begin(uint8_t pin, uint16_t max_throttle) {
    _pin = pin;

    if (MAX_PWM <= MIN_PWM)
      return false;

    // Что то не так
    if (max_throttle > this->max_throttle_m)
      return false;

    this->max_throttle_m = max_throttle;
    is_calibration_done = false;
    _servo.attach(_pin, MIN_PWM, MAX_PWM);
    _servo.setAutoDetach(false);
    _servo.start();
    go(MIN_PWM);

    return true;
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

  void go(const uint16_t pwm) {
    this->pwm = constrain(pwm, MIN_PWM, MIN_PWM + max_throttle_m);
    _servo.writeMicroseconds(this->pwm);
    _servo.tickManual();
  }

  void stop() {
    uint8_t steps = 10;
    uint16_t _pwm = int(max_throttle_m / float(steps));
    for (size_t i = 0; i < steps; i++)
    {
      go(pwm - _pwm);
      delay(100);
    }
    go(MIN_PWM);
  }
};