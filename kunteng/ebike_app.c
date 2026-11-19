/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "ebike_app.h"

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "main.h"
#include "interrupts.h"
#include "adc.h"
#include "utils.h"
#include "motor.h"
#include "pwm.h"
#include "uart.h"
#include "brake.h"
#include "eeprom.h"
#include "settings.h"
#include "display.h"

// cruise control variables
uint8_t ui8_cruise_state = 0;
uint8_t ui8_cruise_output = 0;
uint8_t ui8_cruise_counter = 0;
uint8_t ui8_cruise_value = 0;

// communications variables
volatile struc_lcd_configuration_variables lcd_configuration_variables;
volatile float f_controller_max_current;
float f_wheel_speed;
float f_wheel_perimeter = 2.0625; // 26'' wheel

uint8_t ui8_adc_throttle_value;
uint8_t ui8_adc_throttle_value_cruise_control;
uint8_t ui8_throttle_value;
uint16_t ui16_throttle_value_accumulated = 0;
uint8_t ui8_throttle_value_filtered;
uint8_t ui8_is_throotle_released;
uint8_t ui8_throttle_error_counter = 0;

volatile uint16_t ui16_pas_pwm_cycles_ticks = 0xffff; // "not pedalling" until the first PAS pulses
volatile uint8_t ui8_pas_direction = 0;
uint8_t ui8_pas_cadence_rpm = 0;

volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks = (uint16_t) WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS;
volatile uint8_t ui8_wheel_speed_sensor_is_disconnected = 1; // until the first pulse (was 0: the display showed ~850 km/h for 4 s after power on)

uint16_t ui16_motor_controller_max_current_10b;

// assist level of the display (0 ... 5) -> power factor from the settings
static float assist_level_factor (uint8_t ui8_level)
{
  if (ui8_level == 0) { return PAS_ASSIST_LEVEL_0; }
  if (ui8_level > 5) { ui8_level = 5; }
  return ((float) kt_settings.ui8_assist_percent[ui8_level - 1]) * 0.01;
}

// function prototypes
static void communications_controller (void);
uint8_t ebike_app_cruise_control (uint8_t ui8_value);
void set_speed_erps_max_to_motor_controller (struc_lcd_configuration_variables *lcd_configuration_variables);
void set_motor_controller_max_current (uint8_t ui8_controller_max_current);
void calc_wheel_speed (void);
void ebike_throotle_type_throotle_pas (void);
void ebike_throotle_type_torque_sensor (void);
void read_throotle (void);
void read_pas_cadence_and_direction (void);

void ebike_app_controller (void)
{
  // calc wheel speed and save the value on global variable ui8_wheel_speed
  calc_wheel_speed ();

  // map throttle value from 0 up to 255 to global variable: ui8_throttle_value
  // setup ui8_is_throotle_released flag
  read_throotle ();

  // read PAS cadence to global variable: ui8_pas_cadence_rps
  read_pas_cadence_and_direction ();

  // send and received information to/from the LCD as also setup the configuration variables
  communications_controller ();

  // control the motor using specific algorithm
  if (kt_settings.ui8_throttle_type == EBIKE_THROTTLE_TYPE_TORQUE_SENSOR) { ebike_throotle_type_torque_sensor (); }
  else { ebike_throotle_type_throotle_pas (); }
}

// cruise control will save throttle value and use it even if user releases throttle
// user must keep throttle in the same position for over 8 seconds to start the cruise control
uint8_t ebike_app_cruise_control (uint8_t ui8_value)
{
  // reset cruise control if wheel speed is less than 6km/h
  if (ui8_ebike_app_get_wheel_speed () < 6)
  {
    ui8_cruise_state = 0;
    ui8_cruise_counter = 0;
    return ui8_value;
  }

  switch (ui8_cruise_state)
  {
    case 0:
    if ((ui8_value > CRUISE_CONTROL_MIN) &&
	(((int16_t) ui8_value > ((int16_t) ui8_cruise_value - CRUISE_CONTROL_MIN)) && ((int16_t) ui8_value < ((int16_t) ui8_cruise_value + CRUISE_CONTROL_MIN))))
    {
      ui8_cruise_counter++;
      ui8_cruise_output = ui8_value;

      if (ui8_cruise_counter > 80) // 80 * 100ms = 8 seconds: time to lock cruise control
      {
	ui8_cruise_state = 1;
	ui8_cruise_output = ui8_value;
	ui8_cruise_counter = 0;
	ui8_cruise_value = 0;
      }
    }
    else
    {
      ui8_cruise_counter = 0;
      ui8_cruise_value = ui8_value;
      ui8_cruise_output = ui8_cruise_value;
    }
    break;

    case 1:
    if (ui8_value < CRUISE_CONTROL_MIN) { ui8_cruise_state = 2; }
    break;

    case 2:
    if (ui8_value > CRUISE_CONTROL_MIN)
    {
      ui8_cruise_state = 0;
      ui8_cruise_output = ui8_value;
    }
    break;
  }

  return ui8_cruise_output;
}

uint8_t ebike_app_cruise_control_is_set (void)
{
  return (ui8_cruise_state ? 1: 0);
}

void ebike_app_cruise_control_stop (void)
{
  ui8_cruise_state = 0;
}

uint8_t throttle_is_set (void)
{
  return (ui8_adc_throttle_value > ADC_THROTTLE_MIN_VALUE) ? 1: 0;
}

// the display protocols (Kunteng, KingMeter, Bafang) are in display.c
static void communications_controller (void)
{
  display_controller ();
  set_motor_controller_max_current (lcd_configuration_variables.ui8_controller_max_current);
  set_speed_erps_max_to_motor_controller (&lcd_configuration_variables);
}

void set_speed_erps_max_to_motor_controller (struc_lcd_configuration_variables *lcd_configuration_variables)
{
  uint32_t ui32_temp;
  float f_temp;

  switch (lcd_configuration_variables->ui8_wheel_size)
  {
    case 0x12: // 6''
    f_wheel_perimeter = 0.46875;
    break;

    case 0x0a: // 8''
    f_wheel_perimeter = 0.62847;
    break;

    case 0x0e: // 10''
    f_wheel_perimeter = 0.78819;
    break;

    case 0x02: // 12''
    f_wheel_perimeter = 0.94791;
    break;

    case 0x06: // 14''
    f_wheel_perimeter = 1.10764;
    break;

    case 0x00: // 16''
    f_wheel_perimeter = 1.26736;
    break;

    case 0x04: // 18''
    f_wheel_perimeter = 1.42708;
    break;

    case 0x08: // 20''
    f_wheel_perimeter = 1.57639;
    break;

    case 0x0c: // 22''
    f_wheel_perimeter = 1.74305;
    break;

    case 0x10: // 24''
    f_wheel_perimeter = 1.89583;
    break;

    case 0x14: // 26''
    f_wheel_perimeter = 2.0625;
    break;

    case 0x18: // 700c
    f_wheel_perimeter = 2.17361;
    break;

    case 0x1c: // 28''
    f_wheel_perimeter = 2.19444;
    break;

    case 0x1e: // 29''
    f_wheel_perimeter = 2.25;
    break;

    default: // 26''
    f_wheel_perimeter = 2.0625;
    break;
  }

  // (ui8_max_speed * 1000 * (ui8_motor_characteristic / 2)) / (3600 * f_wheel_size)
  ui32_temp = ((uint32_t) lcd_configuration_variables->ui8_max_speed) * 1000; // in meters/hour
  ui32_temp *= ((uint32_t) (lcd_configuration_variables->ui8_motor_characteristic >> 1));
  // KingMeter displays send the wheel circumference in mm
  if (ui16_display_wheel_perimeter_mm) { f_wheel_perimeter = ((float) ui16_display_wheel_perimeter_mm) * 0.001; }
  f_temp = 3600.0 * f_wheel_perimeter;
  f_temp = ((float) ui32_temp) / f_temp;
  motor_controller_set_speed_erps_max ((uint16_t) f_temp);
}

void set_motor_controller_max_current (uint8_t ui8_controller_max_current)
{
  float f_controller_max_current;

  switch (ui8_controller_max_current)
  {
    case 0:
    f_controller_max_current = 0.1;
    break;

    case 1:
    f_controller_max_current = 0.25;
    break;

    case 2:
    f_controller_max_current = 0.33;
    break;

    case 3:
    f_controller_max_current = 0.5;
    break;

    case 4:
    f_controller_max_current = 0.667;
    break;

    case 5:
    f_controller_max_current = 0.752;
    break;

    case 6:
    f_controller_max_current = 0.8;
    break;

    case 7:
    f_controller_max_current = 0.833;
    break;

    case 8:
    f_controller_max_current = 0.87;
    break;

    case 9:
    f_controller_max_current = 0.91;
    break;

    case 10:
    f_controller_max_current = 1.0;
    break;

    default:
    f_controller_max_current = 1.0;
    break;
  }

  ui16_motor_controller_max_current_10b = (uint16_t) (((float) (kt_settings.ui8_motor_current_a * 8)) * f_controller_max_current);
}

struc_lcd_configuration_variables *ebike_app_get_lcd_configuration_variables (void)
{
  return &lcd_configuration_variables;
}

uint8_t ebike_app_get_adc_throttle_value_cruise_control (void)
{
  return ui8_adc_throttle_value_cruise_control;
}

uint8_t ebike_app_is_throttle_released (void)
{
  return ui8_is_throotle_released;
}

uint8_t ui8_ebike_app_get_wheel_speed (void)
{
  return f_wheel_speed;
}

void calc_wheel_speed (void)
{
  uint32_t ui32_temp;
  uint32_t ui32_temp1;

  if (ui8_wheel_speed_sensor_is_disconnected)
  {
    // calc wheel speed in km/h, from motor hall sensors signals
    ui32_temp = ((uint32_t) (lcd_configuration_variables.ui8_motor_characteristic >> 1)) * 1000;
    ui32_temp1 = ((uint32_t) ui16_motor_get_motor_speed_erps ()) * 3600;
    f_wheel_speed = ((float) ui32_temp1) * f_wheel_perimeter;
    f_wheel_speed /= (float) ui32_temp;
  }
  else
  {
    // calc wheel speed in km/h, from external wheel speed sensor
    f_wheel_speed = ((float) PWM_CYCLES_SECOND) / ((float) ui16_wheel_speed_sensor_pwm_cycles_ticks); // rps
    f_wheel_speed *= f_wheel_perimeter; // meters per second
    f_wheel_speed *= 3.6; // kms per hour
  }
}

void read_pas_cadence_and_direction (void)
{
  // cadence in RPM =  60 / (ui16_pas_timer2_ticks * PAS_NUMBER_MAGNETS * 0.000064)
  if (ui16_pas_pwm_cycles_ticks >= kt_settings.ui16_pas_min_cadence_ticks) { ui8_pas_cadence_rpm = 0; }
  else
  {
    ui8_pas_cadence_rpm = (uint8_t) (60 / (((float) ui16_pas_pwm_cycles_ticks) * ((float) kt_settings.ui8_pas_magnets) * 0.000064));

    if (ui8_pas_cadence_rpm > kt_settings.ui8_pas_max_cadence)
    {
      ui8_pas_cadence_rpm = kt_settings.ui8_pas_max_cadence;
    }
  }

  if (ui8_pas_direction) { ui8_pas_cadence_rpm = 0; }
}

void ebike_throotle_type_throotle_pas (void)
{
#if defined (EBIKE_THROTTLE_TYPE_THROTTLE_PAS_PWM_DUTY_CYCLE)
  uint8_t ui8_temp;
  float f_temp;

  // set target motor speed to the value defined on the LCD
  // (due to motor configurations on the motor controller, this will only put a limit to the max permited speed!)
  motor_controller_set_target_speed_erps (motor_controller_get_target_speed_erps_max ());

  // map ui8_pas_cadence_rpm to 0 - 255
  ui8_temp = (uint8_t) (map ((uint32_t) ui8_pas_cadence_rpm,
		 (uint32_t) 0,
		 (uint32_t) kt_settings.ui8_pas_max_cadence,
		 (uint32_t) 0,
		 (uint32_t) 255));

  f_temp = (float) (((float) ui8_temp) * ((float) lcd_configuration_variables.ui8_assist_level) * 0.2);
  ui8_temp = (uint8_t) (map ((uint32_t) f_temp,
  			 (uint32_t) 0,
  			 (uint32_t) 255,
  			 (uint32_t) 0,
  			 (uint32_t) PWM_DUTY_CYCLE_MAX));
  ui16_pwm_duty_cycle_duty_cycle_controller = ((uint16_t) ui8_temp) << 1; // 9 bit duty cycle

#elif defined (EBIKE_THROTTLE_TYPE_THROTTLE_PAS_CURRENT_SPEED)
  uint8_t ui8_temp;
  uint16_t ui16_temp;
  float f_temp;
  uint16_t ui16_target_speed_erps;

  // map ui8_pas_cadence_rpm to 0 - 255
  ui8_temp = (uint8_t) (map ((uint32_t) ui8_pas_cadence_rpm,
		 (uint32_t) 0,
		 (uint32_t) kt_settings.ui8_pas_max_cadence,
		 (uint32_t) 0,
		 (uint32_t) 255));

  ui8_temp = ui8_max (ui8_throttle_value_filtered, ui8_temp); // use the max value from throotle or pas cadence

  // scale with assist level value
  f_temp = assist_level_factor (lcd_configuration_variables.ui8_assist_level);

  f_temp = (float) (((float) ui8_temp) * ((float) lcd_configuration_variables.ui8_assist_level) * f_temp);

  // map to motor controller current
  ui16_temp = (uint16_t) (map ((uint32_t) f_temp,
		   (uint32_t) 0,
		   (uint32_t) 255,
		   (uint32_t) 0,
		   (uint32_t) ui16_motor_controller_max_current_10b));
  motor_controller_set_target_current_10b (ui16_temp);

  // if LCD P3 = 0, control also the speed
  if (lcd_configuration_variables.ui8_power_assist_control_mode)
  {
    // set target motor speed to the value defined on the LCD
    // (due to motor configurations on the motor controller, this will only put a limit to the max permited speed!)
    motor_controller_set_target_speed_erps (motor_controller_get_target_speed_erps_max ());
  }
  else
  {
    // PAS cadence will setup motor speed
    // map PAS candence value to motor speed value
    ui16_target_speed_erps = (uint16_t) (map ((uint32_t) f_temp,
		  (uint32_t) 0,
		  (uint32_t) 255,
		  (uint32_t) 0, // motor speed min value
		  (uint32_t) motor_controller_get_target_speed_erps_max ())); // motor speed from max value, defined on the LCD
    motor_controller_set_target_speed_erps (ui16_target_speed_erps);
  }
#endif
}

void ebike_throotle_type_torque_sensor (void)
{
  uint16_t ui16_target_current_10b;
  uint16_t ui16_temp;
  float f_temp;
  uint16_t ui16_target_speed_erps;

  f_temp = assist_level_factor (lcd_configuration_variables.ui8_assist_level);

  f_temp = (float) (((float) ui8_throttle_value_filtered) * (f_temp / 2.0));

#if defined (EBIKE_THROTTLE_TYPE_TORQUE_SENSOR_HUMAN_POWER)
  // calc humam power on the crank using as input the pedal torque sensor value and pedal cadence
  ui16_temp = (uint16_t) (f_temp * ((float) ((float) ui8_pas_cadence_rpm / ((float) kt_settings.ui8_pas_max_cadence))));
#else
  ui16_temp = (uint16_t) f_temp;
#endif

  ui16_target_current_10b = (uint16_t) (map ((uint32_t) ui16_temp, // human power value
			   (uint32_t) 0, // min input value
			   (uint32_t) 255, // max input value
			   (uint32_t) 0, // min output motor current value
			   (uint32_t) ui16_motor_controller_max_current_10b));  // max output motor current value
  motor_controller_set_target_current_10b (ui16_target_current_10b);

  // if LCD P3 = 0, control also the speed
  if (lcd_configuration_variables.ui8_power_assist_control_mode)
  {
    // set target motor speed to the value defined on the LCD
    // (due to motor configurations on the motor controller, this will only put a limit to the max permited speed!)
    motor_controller_set_target_speed_erps (motor_controller_get_target_speed_erps_max ());
  }
  else
  {
    // humam power will setup motor speed
    // map humam power value to motor speed value
    ui16_target_speed_erps = (uint16_t) (map ((uint32_t) ui16_temp,
		  (uint32_t) 0,
		  (uint32_t) 255,
		  (uint32_t) 0, // motor speed min value
		  (uint32_t) motor_controller_get_target_speed_erps_max ())); // motor speed from max value, defined on the LCD
    motor_controller_set_target_speed_erps (ui16_target_speed_erps);
  }
}

void read_throotle (void)
{
  // read torque sensor signal
  ui8_adc_throttle_value = ui8_adc_read_throttle ();

  // a reading above ADC_THROTTLE_MAX_ERROR means a broken ground wire or a short to 5 V, not full throttle:
  // ignore the throttle and show error 01 until the signal is back in range for 1 s
  if (ui8_adc_throttle_value > ADC_THROTTLE_MAX_ERROR)
  {
    ui8_throttle_error_counter = 10;
    motor_controller_set_error (MOTOR_CONTROLLER_ERROR_01_THROTTLE);
  }
  else if (ui8_throttle_error_counter)
  {
    if (--ui8_throttle_error_counter == 0) { motor_controller_clear_error (); }
  }
  if (ui8_throttle_error_counter)
  {
    ui8_adc_throttle_value = 0;
    ui16_throttle_value_accumulated = 0;
  }

  // map throttle value from 0 up to 255
  ui8_throttle_value = (uint8_t) (map (ui8_adc_throttle_value, ADC_THROTTLE_MIN_VALUE, ADC_THROTTLE_MAX_VALUE, 0, 255));

  // low pass filter the torque sensor to smooth the signal
  ui16_throttle_value_accumulated -= ui16_throttle_value_accumulated >> 2;
  ui16_throttle_value_accumulated += ((uint16_t) ui8_throttle_value);
  ui8_throttle_value_filtered = ui16_throttle_value_accumulated >> 2;

  // setup ui8_is_throotle_released flag
  ui8_is_throotle_released = (ui8_throttle_value ? 0 : 1);
}
