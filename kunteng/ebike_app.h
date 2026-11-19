/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EBIKE_APP_H_
#define _EBIKE_APP_H_

#include <stdint.h>
#include "main.h"

typedef struct _lcd_configuration_variables
{
  uint8_t ui8_assist_level;
  uint8_t ui8_motor_characteristic;
  uint8_t ui8_wheel_size;
  uint8_t ui8_max_speed;
  uint8_t ui8_power_assist_control_mode;
  uint8_t ui8_controller_max_current;
} struc_lcd_configuration_variables;

extern volatile uint16_t ui16_pas_pwm_cycles_ticks;
extern volatile uint8_t ui8_pas_direction;

extern volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks;
extern volatile uint8_t ui8_wheel_speed_sensor_is_disconnected;

extern uint8_t ui8_throttle_value;
extern uint8_t ui8_adc_throttle_value;

void ebike_app_controller (void);
void ebike_app_cruise_control_stop (void);
uint8_t ebike_app_get_adc_throttle_value_cruise_control (void);
struc_lcd_configuration_variables *ebike_app_get_lcd_configuration_variables (void);
uint8_t ebike_app_is_throttle_released (void);
uint8_t ui8_ebike_app_get_wheel_speed (void);

#endif /* _EBIKE_APP_H_ */
