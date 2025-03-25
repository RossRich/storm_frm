#include <Arduino.h>
#include <AsyncStream.h>
#include <GyverFilters.h>
#include <GyverHX711.h>
#include <StringUtils.h>
#include <TimerMs.h>
#include "ACS758_50B.hpp"
#include "CMD_CODE.h"
#include "Motor.hpp"
#include "Volt.hpp"

#define DEBUG_DATA 0

#define ESC_PIN 6u  //< ШИМ пин для мотора

#define SERIAL_MODE_WEB 0u
#define SERIAL_MODE_TER 1u

#define DISARM 0u;
#define ARM 1u;

#define HX_DT_PIN 3         //< Тензодатчик
#define HX_SCK_PIN 2        //< Тензодатчик
#define HX_SCALE 201100.0f  //< Коэффициен для весов
// #define HX_SCALE (49.98f * 1000.0f) //< Коэффициен для весов

#define VOLTAGE_SENSOR_PIN A1
#define CURRENT_SENSOR_PIN A0  //< ACS758

#define LOOP_RATE 15u
#define UPDATA_DATA_RATE 15u
#define COMMUNICATION_RATE 3u

#define NUM_STEEPS 10u  //< Кол-во этапов измерения
#define RUN_PERIOD \
  3000u  //< время замера этапа. Общее время измерения = NUM_STEEPS * RUN_PERIOD

#define LED_ON (digitalWrite(LED_BUILTIN, HIGH))
#define LED_OFF (digitalWrite(LED_BUILTIN, LOW))

#define PAYLOAD_SIZE 35u

static char buf[PAYLOAD_SIZE];

enum STATES : uint8_t {
  INIT = 0,
  STREAMING,
  RESET,
  COMMUNICATION,
  RUN_TEST,
  SETUP_TEST,
  CALIBRATION,
  STOP_TEST,
  RECEIVE_BEGIN,
  RECEIVE_SETUP,
  SEND_SETUP
} typedef state_t;

typedef struct Setup {
  uint16_t max_throttle;
  uint16_t max_pwm;
  uint16_t min_pwm;
} setup_t;

typedef struct Command {
  uint8_t value;
} command_t;

static setup_t sys_setup = {.max_throttle = 1000,
                            .max_pwm = MAX_PWM,
                            .min_pwm = MIN_PWM};

static float weight = 0;
static float current = 0.0;
static float voltage = 0.0;
static uint16_t run_steep = NUM_STEEPS;
static state_t now_state = STATES::INIT;
static state_t state_backup = now_state;
static uint8_t serial_mode = SERIAL_MODE_WEB;
volatile static uint8_t is_arm = DISARM;

AsyncStream<50> async_stream(&Serial, '\n');
GFilterRA weight_filter(0.35);
GFilterRA current_filter(0.35);
GyverHX711 weight_sensor(HX_DT_PIN, HX_SCK_PIN);
ACS758_50B current_sensor;
Volt voltage_sensor;
Motor motor;

TimerMs run_timer;
TimerMs loop_timer(1000 / LOOP_RATE);
TimerMs cmn_timer(1000 / COMMUNICATION_RATE);
TimerMs update_data_timer(1000 / UPDATA_DATA_RATE, 1, 0);
TimerMs receive_timer;

void trs(state_t new_state) {
  now_state = new_state;
}

void setup_hx() {
  weight_sensor.sleepMode(false);
  weight_sensor.setChannel(HX_GAIN64_A);
  while (not weight_sensor.available() or weight_sensor.read() == 0) {
  }

  uint64_t time = millis() + 1000;
  while (time > millis()) {
    weight_sensor.read();
  }

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

// TODO: 2 знака после запятой
void data_to_serial() {
  if (serial_mode == SERIAL_MODE_WEB) {
    sprintf(buf, "$D;%u;%i;%i;%i;%u!\n", now_state, int(weight * 1000),
            int(current * 10), int(voltage * 10), motor.pwm);

    // uint8_t data_len = strlen(buf+2) - 1;

    // if (data_len > 254) {
    // Serial.print("Invalid data len ");Serial.println(int(data_len));
    // return;
    // }

    // buf[0] = '$';
    // buf[1] = data_len;

    if (Serial.availableForWrite())
      Serial.write(buf, strlen(buf));
  } else {
    Serial.println("Not implimented");
  }
}

void setup_to_serial() {
  sprintf(buf, "$S;%u;%u;%u!\n", motor.max_throttle_m, MAX_PWM, MIN_PWM);

  // uint8_t data_len = strlen(buf+2) - 1;

  // if (data_len > 254) {
  // Serial.print("Invalid data len ");Serial.println(int(data_len));
  // return;
  // }
  // $C;123!
  // buf[0] = '$';
  // buf[1] = data_len;

  if (Serial.availableForWrite())
    Serial.write(buf, strlen(buf));
}

void reset() {
  motor.pwm = MIN_PWM;
  weight = 0.0f;
  current = 0.0f;
}

void update_data() {
  if (not update_data_timer.tick())
    return;

#if DEBUG_DATA
  weight = 5.250f;
  current = 10.0f;
  voltage = 22.5f;
#else
  if (weight_sensor.available())
    weight = weight_filter.filtered(weight_sensor.read() / HX_SCALE);

  current = current_filter.filtered(current_sensor.current());
  voltage = voltage_sensor.value();
#endif
}

void setup() {
  Serial.begin(115200);

  setup_led();
  LED_ON;

  now_state = STATES::INIT;

  if (not current_sensor.begin(CURRENT_SENSOR_PIN)) {
    Serial.println("Invalid current sensor");
    for (;;) {
    }
  }

  if (not voltage_sensor.begin(VOLTAGE_SENSOR_PIN)) {
    Serial.println("Invalid voltage sensor");
    for (;;) {
    }
  }

  if (not motor.begin(ESC_PIN, (MAX_PWM - MIN_PWM) - TRANK_PWM)) {
    Serial.println("Setup motor failed");
    for (;;) {
    }
  }

  // setup_hx();

  setup_to_serial();
  delay(100);
  setup_to_serial();
  delay(100);
  setup_to_serial();
  delay(100);

  loop_timer.setPeriodMode();
  loop_timer.start();

  cmn_timer.setPeriodMode();
  cmn_timer.start();

  LED_OFF;
}

bool update_motor_params(const setup_t& new_params) {
  if (new_params.min_pwm > new_params.max_pwm or
      new_params.max_pwm == new_params.min_pwm)
    return false;

  sys_setup.max_pwm = new_params.max_pwm;
  sys_setup.min_pwm = new_params.min_pwm;

  uint16_t std_throttle = sys_setup.max_pwm - sys_setup.min_pwm;
  if (new_params.max_throttle == 0 or new_params.max_throttle > std_throttle)
    return false;

  sys_setup.max_throttle = new_params.max_throttle;

  return true;
}

void on_new_cmd(command_t cmd) {
  Serial.print(F("Command: "));
  Serial.println(cmd.value);

  if (cmd.value == START_TEST_CODE) {
    if (now_state == STATES::STREAMING)
      trs(STATES::SETUP_TEST);
  } else if (cmd.value == START_CALIB_CODE) {
    if (now_state == STREAMING)
      trs(STATES::CALIBRATION);
  } else if (cmd.value == STOP_TEST_CODE) {
    trs(STATES::STOP_TEST);
  } else if (cmd.value == GET_SETUP) {
    trs(STATES::SEND_SETUP);
  } else if (cmd.value == SET_SETUP) {
    trs(STATES::RECEIVE_BEGIN);
  }
}

void on_new_setup(const setup_t& setup) {
  if (update_motor_params(setup)) {
    motor.max_throttle_m = sys_setup.max_throttle;
    Serial.println("New setup for motor");
  }
}
// $C;30!
// $S;1000;2000;1000!
void parse_mag(Text& msg) {
  static constexpr char _cmd = 'C';
  static constexpr char _setup = 'S';

  uint16_t num = msg.count(";");
  Text params[num];
  auto nn = msg.split(params, msg.length(), ';');

  const char msg_type = *params[0].c_str();

  if (msg_type == _cmd) {
    static command_t tmp_cmd;
    tmp_cmd.value = atoi(params[1].c_str());
    on_new_cmd(tmp_cmd);
  } else if (msg_type == _setup) {
    static setup_t tmp_setup;
    tmp_setup.max_throttle = params[1];
    tmp_setup.max_pwm = params[2];
    tmp_setup.min_pwm = params[3];
    on_new_setup(tmp_setup);
  }
}

void check_port() {
  //< timout ломает чтение
  // if (not cmn_timer.tick())
  // return;

  if (not async_stream.available())
    return;

  LED_ON;

  Text data(async_stream.buf, strlen(async_stream.buf));
  int16_t start_idx = data.indexOf('$');
  int16_t end_idx = data.indexOf('!', start_idx + 1);

  if (start_idx == -1 or end_idx == -1)
    return;

  if (start_idx > end_idx)
    return;

  Text data2 = data.substring(start_idx + 1, end_idx);
  parse_mag(data2);

  // memset(async_stream.buf, 0, data.length());

  LED_OFF;
}

void loop() {
  check_port();
  update_data();

  if (!loop_timer.tick())
    return;

  switch (now_state) {
    case STATES::INIT:
      trs(STREAMING);
      break;

    case STATES::STREAMING:
      // data_to_serial();
      break;

    case STATES::SETUP_TEST:
      led_blink(250);
      LED_ON;

      run_steep = 1;
      run_timer.setTime(RUN_PERIOD);
      run_timer.setTimerMode();
      trs(STATES::RUN_TEST);
      run_timer.start();
    case STATES::RUN_TEST:
      if (!run_timer.tick()) {
        motor.pwm = MIN_PWM + static_cast<uint16_t>(motor.max_throttle_m /
                                                    NUM_STEEPS * run_steep);
        motor.go(motor.pwm);
      } else {
        if (run_steep < NUM_STEEPS) {
          run_steep += 1;
          run_timer.start();
        } else {
          trs(STATES::STOP_TEST);
        }
      }
      break;

    case STATES::STOP_TEST:
      motor.stop();
      if (motor.pwm == MIN_PWM)
        trs(STATES::STREAMING);
      LED_OFF;
      break;

    case STATES::CALIBRATION:
      led_blink(250);
      LED_ON;
      motor.calibrate();
      trs(STATES::STREAMING);
      LED_OFF;
      break;

    case STATES::SEND_SETUP:
      setup_to_serial();
      delay(500);
      trs(STATES::STREAMING);
      break;

    default:
      break;
  };

  // update_data();
  data_to_serial();
}