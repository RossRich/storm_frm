#include <Wire.h>
#include <ACS758_50B.hpp>
#include <Arduino.h>
#include <GyverFilters.h>
#include <GyverHX711.h>
#include <TimerMs.h>
#include <ServoSmooth.h>

#define ESC_PIN 6u
#define MAX_PWM 2000u
#define MIN_PWM 1000u

#define HX_DT_PIN 3           //< Тензодатчик
#define HX_SCK_PIN 2          //< Тензодатчик
#define HX_SCALE 49.98f //< Коэффициен для весов

#define CURRENT_SENSOR_PIN A0 //< ACS758
#define VCC_CUSTOM_PIN 4

#define LOOP_RATE 60u
#define UPDATA_DATA_RATE 10u
#define COMMUNICATION_RATE 1u

#define NUM_STEEPS 3u //< Кол-во этапов измерения
#define RUN_PERIOD 3000u //< время замера этапа. Общее время измерения = NUM_STEEPS * RUN_PERIOD

#define LED_ON (digitalWrite(LED_BUILTIN, HIGH))
#define LED_OFF (digitalWrite(LED_BUILTIN, LOW))

enum STATES: uint8_t {
  INIT = 0,
  STREAMING,
  RESET,
  COMMUNICATION,
  RUN_TEST,
  SETUP_TEST,
} typedef state_t;

static struct Motor
{
  uint16_t pwm = MIN_PWM;
  uint16_t max_throttle = MAX_PWM;
} motor;


static long weight = 0;
static float current = 0.0;
static float voltage = 0.0;
static uint16_t run_steep = NUM_STEEPS;
static state_t now_state = STATES::INIT;

void trs(state_t new_state) {
  now_state = new_state;
}

GyverHX711 weight_sensor(HX_DT_PIN, HX_SCK_PIN);
// GFilterRA weight_filter(0.4);
ACS758_50B current_sensor;
TimerMs run_timer;
TimerMs loop_timer(1000 / LOOP_RATE);
TimerMs cmn_timer(1000 / COMMUNICATION_RATE);
TimerMs update_data_timer(1000 / UPDATA_DATA_RATE, 1, 0);

ServoSmooth servo;

void setup_hx() {
  weight_sensor.sleepMode(false);
  delay(100);
  for (size_t i = 0; i < 25; i++)
  {
    if (weight_sensor.available())
      weight = weight_sensor.read();

    delay(100);
  }

  weight_sensor.setOffset(-weight);
  weight_sensor.tare();
  weight = 0;
}

void setup_led() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup_esc() {

  motor.max_throttle = 500;
  motor.pwm = MIN_PWM;

  servo.attach(ESC_PIN, MIN_PWM, MAX_PWM);
  servo.setAutoDetach(false);
  servo.start();

  delay(3000);
  LED_ON;
  servo.writeMicroseconds(MAX_PWM);
  servo.tickManual();
  delay(6000);
  LED_OFF;
  servo.writeMicroseconds(MIN_PWM);
  servo.tickManual();
  delay(500);
  LED_ON;
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
  voltage = (analogRead(A1) * (VCC_REF / float(ADC_MAX))) / 0.09856f;
}


void setup() {
  Serial.begin(115200);

  setup_led();

  now_state = STATES::INIT;
  // pinMode(VCC_CUSTOM_PIN, OUTPUT);
  // digitalWrite(VCC_CUSTOM_PIN, HIGH);
  current_sensor.begin(CURRENT_SENSOR_PIN);
  pinMode(A1, INPUT);
  setup_hx();

  data_to_serail();
  delay(100);
  data_to_serail();
  delay(100);
  data_to_serail();

  setup_esc();
  
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

  switch (now_state)
  {
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
      motor.pwm = MIN_PWM + static_cast<uint16_t>(motor.max_throttle / run_steep);
      servo.writeMicroseconds(motor.pwm);
    } else {
      if (run_steep > 1) {
        run_steep -= 1;
        run_timer.start();
      } else {
        motor.pwm = MIN_PWM;
        servo.writeMicroseconds(motor.pwm);
        trs(STATES::STREAMING);
        LED_OFF;
      }
    }
    servo.tick();
    break;
  
  default:
    break;
  };
  
  update_data();
  data_to_serail();
}