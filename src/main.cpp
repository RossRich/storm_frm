#include "ACS758_50B.hpp"
#include "Motor.hpp"
#include "Volt.hpp"
#include <Arduino.h>
#include <GyverFilters.h>
#include <GyverHX711.h>
#include <TimerMs.h>

#define ESC_PIN 6u //< ШИМ пин для мотора

#define HX_DT_PIN 3     //< Тензодатчик
#define HX_SCK_PIN 2    //< Тензодатчик
#define HX_SCALE 49.98f //< Коэффициен для весов

#define VOLTAGE_SENSOR_PIN A1
#define CURRENT_SENSOR_PIN A0 //< ACS758

#define LOOP_RATE 1u
#define UPDATA_DATA_RATE 1u
#define COMMUNICATION_RATE 1u

#define NUM_STEEPS 3u //< Кол-во этапов измерения
#define RUN_PERIOD                                                             \
  3000u //< время замера этапа. Общее время измерения = NUM_STEEPS * RUN_PERIOD

#define LED_ON (digitalWrite(LED_BUILTIN, HIGH))
#define LED_OFF (digitalWrite(LED_BUILTIN, LOW))

static char buf[26];

enum STATES : uint8_t {
  INIT = 0,
  STREAMING,
  RESET,
  COMMUNICATION,
  RUN_TEST,
  SETUP_TEST,
} typedef state_t;

static long weight = 0;
static float current = 0.0;
static float voltage = 0.0;
static uint16_t run_steep = NUM_STEEPS;
static state_t now_state = STATES::INIT;

void trs(state_t new_state) { now_state = new_state; }

// GFilterRA weight_filter(0.4);
GyverHX711 weight_sensor(HX_DT_PIN, HX_SCK_PIN);
ACS758_50B current_sensor;
Volt voltage_sensor;
Motor motor;

TimerMs run_timer;
TimerMs loop_timer(1000 / LOOP_RATE);
TimerMs cmn_timer(1000 / COMMUNICATION_RATE);
TimerMs update_data_timer(1000 / UPDATA_DATA_RATE, 1, 0);

void setup_hx() {
  weight_sensor.sleepMode(false);
  while (not weight_sensor.available()) {
  }
  delay(1000);
  weight_sensor.tare();
}

void setup_led() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void led_blink(uint16_t ms = 250) {
  LED_ON;
  delay(ms);
  LED_OFF;
}

void motor_calibration() {
  led_blink();
  motor.calibrate();
}

void data_to_serail() {
  Serial.print(static_cast<uint8_t>(now_state));
  Serial.print(' ');
  Serial.print(weight / HX_SCALE);
  Serial.print(' ');
  Serial.print(current);
  Serial.print(' ');
  Serial.print(voltage);
  Serial.print(' ');
  Serial.print(motor.pwm);
  Serial.print(' ');
  Serial.println();
}

void data_to_serial2() {
  buf[0] = '$';
  buf[1] = now_state + '0';
  buf[2] = ' ';
  dtostrf(weight / HX_SCALE, 4, 2, buf + 3);
  buf[7] = ';';
  buf[8] = '\n';
  buf[9] = '\0';
  Serial.write(buf, strlen(buf));
}

void reset() {
  motor.pwm = MIN_PWM;
  weight = 0;
  current = 0;
  weight_sensor.tare();
}

void update_data() {
  if (not update_data_timer.tick())
    return;

  if (weight_sensor.available())
    weight = weight_sensor.read();

  current = current_sensor.current();
  voltage = voltage_sensor.value();
}

void setup() {
  Serial.begin(115200);

  setup_led();

  now_state = STATES::INIT;

  current_sensor.begin(CURRENT_SENSOR_PIN);
  voltage_sensor.begin(VOLTAGE_SENSOR_PIN);

  setup_hx();

  motor.begin(ESC_PIN, 500);

  data_to_serail();
  delay(100);
  data_to_serail();
  delay(100);
  data_to_serail();

  loop_timer.setPeriodMode();
  loop_timer.start();

  cmn_timer.setPeriodMode();
  cmn_timer.start();

  LED_OFF;
}

void loop() {
  if (!loop_timer.tick())
    return;

  // ввод блокируется пока выполняется тест
  if (now_state != STATES::RUN_TEST and cmn_timer.tick()) {
    trs(STATES::COMMUNICATION);
  }

  switch (now_state) {
  case STATES::INIT:
    trs(STREAMING);
    break;

  case STATES::STREAMING:
    break;

  case STATES::COMMUNICATION:
    if (Serial.available() >= 3) {
      String s = Serial.readStringUntil('\n');
      uint8_t cmd = static_cast<uint8_t>(atoi(s.c_str()));
      if (cmd == 100)
        trs(STATES::SETUP_TEST);
      else
        trs(STATES::STREAMING);
    } else
      trs(STATES::STREAMING);

    break;

  case STATES::SETUP_TEST:
    LED_ON;
    run_steep = NUM_STEEPS;
    run_timer.setTime(RUN_PERIOD);
    run_timer.setTimerMode();
    trs(STATES::RUN_TEST);
    run_timer.start();
  case STATES::RUN_TEST:
    if (!run_timer.tick()) {
      motor.pwm =
          MIN_PWM + static_cast<uint16_t>(motor.max_throttle / run_steep);
      motor.go(motor.pwm);
    } else {
      if (run_steep > 1) {
        run_steep -= 1;
        run_timer.start();
      } else {
        motor.stop();
        trs(STATES::STREAMING);
        LED_OFF;
      }
    }
    // servo.tick();
    break;

  default:
    break;
  };

  update_data();
  data_to_serail();
  // data_to_serial2();
}