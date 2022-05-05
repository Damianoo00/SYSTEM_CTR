#include "../include/Control.h"
#include "../include/PWM.h"
#include "../include/sensors.h"
#include "../include/uart.h"
#include "../include/i2c.h"
#include "../include/converters.h"
#include <Arduino.h>

#define LOG
#define WORK

/***** POUT *****/
#define PWM1_DC 11
#define PWM2_DC 10
#define CURR_PORT A0
#define VOLT_SENSOR A1
#define PWM_SC 6

/*** UART params***/
constexpr long BAUD = 115200;
constexpr int TIMEOUT = 10; // ms

/*** SC params ***/
constexpr int SC_voltage = 2500; // mV

/*** I2C slave id ***/
#define ENCODER_ID 8

/*** REG I params ***/
constexpr float Ts = 10e3;
constexpr float Kr_i = 3.2593;
constexpr float Tr_i = 4.6136;
constexpr int8_t max_i = 1;
constexpr int8_t min_i = -1;

/*** REG V params ***/
constexpr float Kr_v = 2.8e-5;
constexpr float Tr_v = 1.5e-3;
constexpr int8_t max_v = 126;
constexpr int8_t min_v = -126;

struct PICTRL PIctrl_curr;
struct PICTRL PIctrl_speed;

/* REF speed values */
constexpr int speed_ref = 300;    // rad/s
constexpr int curr_ref = 105;     // A
constexpr int voltage_ref = 5000; // mV

void setup()
{
  uart_begin(BAUD, TIMEOUT);
  i2c_begin_master();

  PWM_begin(PWM1_DC);
  PWM_begin(PWM2_DC);
  PWM_begin(PWM_SC);

  InitPIctrl(&PIctrl_speed, Ts, Kr_v, Tr_v, max_v, min_v);
  InitPIctrl(&PIctrl_curr, Ts, Kr_i, Tr_i, max_i, min_i);
}

void loop()
{
  static int curr_sensor = 0;
  static int speed_sensor = 0;
  static int pv_voltage = 0;

#ifdef WORK
  curr_sensor = read_current(CURR_PORT, 1);
  speed_sensor = i2c_get_value_from_slave(ENCODER_ID, 4);
  pv_voltage = get_voltage(VOLT_SENSOR);
#endif

#ifdef SET_CURR
  curr_sensor = uart_recive();
#endif
  CalcPIctrl(&PIctrl_speed, speed_ref - speed_sensor);
  CalcPIctrl(&PIctrl_curr, PIctrl_speed.y - speed_sensor);

  PWM_write(PWM_SC, get_Cuk_duty(voltage_ref - pv_voltage, SC_voltage));
  PWM_write(PWM1_DC, PIctrl_curr.y);
  PWM_write(PWM2_DC, -PIctrl_curr.y);

#ifdef LOG
  log_uart(millis(), pv_voltage, speed_ref, speed_sensor, curr_ref, curr_sensor, PIctrl_curr.y);
#endif
}