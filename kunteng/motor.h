/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

// motor states
#define BLOCK_COMMUTATION 			1
#define SINEWAVE_INTERPOLATION_60_DEGREES 	2
#define SINEWAVE_INTERPOLATION_360_DEGREES 	3

#define MOTOR_STATE_COAST 	0
#define MOTOR_STATE_STOP 	1
#define MOTOR_STATE_STARTUP 	2
#define MOTOR_STATE_COOL 	3
#define MOTOR_STATE_RUNNING 	4

#define MOTOR_CONTROLLER_STATE_OK		1
#define MOTOR_CONTROLLER_STATE_BRAKE		2
#define MOTOR_CONTROLLER_STATE_OVER_CURRENT	4
#define MOTOR_CONTROLLER_STATE_UNDER_VOLTAGE	8
#define MOTOR_CONTROLLER_STATE_THROTTLE_ERROR	16
#define MOTOR_CONTROLLER_STATE_MOTOR_BLOCKED	32
#define MOTOR_CONTROLLER_STATE_OVER_VOLTAGE	64
#define MOTOR_CONTROLLER_STATE_HALL_ERROR	128

#define MOTOR_CONTROLLER_ERROR_EMPTY			0x0
#define MOTOR_CONTROLLER_ERROR_01_THROTTLE		0x22
#define MOTOR_CONTROLLER_ERROR_02			0x23
#define MOTOR_CONTROLLER_ERROR_03			0x24 // KT display: 24 = motor hall signal fault
#define MOTOR_CONTROLLER_ERROR_HALL		MOTOR_CONTROLLER_ERROR_03
#define MOTOR_CONTROLLER_ERROR_04			0x26
#define MOTOR_CONTROLLER_ERROR_05			0x28
#define MOTOR_CONTROLLER_ERROR_06_SHORT_CIRCUIT		0x21
#define MOTOR_CONTROLLER_ERROR_91_BATTERY_UNDER_VOLTAGE 0x91
#define MOTOR_CONTROLLER_ERROR_OVER_VOLTAGE		MOTOR_CONTROLLER_ERROR_05 // own code, so the display no longer shows "battery empty" for over voltage

extern uint8_t ui8_motor_commutation_type;
extern volatile uint8_t ui8_angle_correction;
extern uint8_t ui8_adc_motor_total_current;
extern uint8_t ui8_motor_total_current_offset;
extern volatile uint16_t ui16_duty_cycle; // 9 bit: 0 ... 508
extern uint16_t ui16_duty_cycle_target;
extern uint16_t ui16_PWM_cycles_counter_total;
extern int16_t i16_motor_current_filtered_10b;
extern uint16_t ui16_pwm_duty_cycle_duty_cycle_controller;
extern volatile int8_t i8_hall_offset[8];

/***************************************************************************************/
// Motor interface
void hall_sensor_init (void); // must be called before using the motor
void motor_init (void); // must be called before using the motor
void motor_controller (void); // every 100 ms
void motor_fast_controller (void); // every 10 ms
void motor_enable_PWM (void);
void motor_disable_PWM (void);
void motor_set_pwm_duty_cycle_target (uint16_t value); // 0 ... 508
void motor_set_current_max (uint8_t value); // steps of 0.5A each step
int16_t motor_get_current_filtered_10b (void); // steps of 0.125A each step
void motor_set_regen_current_max (uint8_t value); // steps of 0.5A each step
void motor_set_pwm_duty_cycle_ramp_up_inverse_step (uint16_t value); // each step = 64us
void motor_set_pwm_duty_cycle_ramp_down_inverse_step (uint16_t value); // each step = 64us
uint16_t ui16_motor_get_motor_speed_erps (void);
uint16_t motor_get_er_PWM_ticks (void); // PWM ticks per electronic rotation
void motor_controller_set_state (uint8_t state);
void motor_controller_reset_state (uint8_t state);
uint8_t motor_controller_get_state (void);
uint8_t motor_controller_state_is_set (uint8_t state);
uint8_t motor_get_ADC_battery_voltage_filtered (void);
void motor_controller_set_target_speed_erps (uint16_t ui16_erps);
void motor_controller_set_speed_erps_max (uint16_t ui16_erps);
uint16_t motor_controller_get_target_speed_erps_max (void);
void motor_controller_set_target_current_10b (uint16_t ui16_current);
void motor_controller_set_error (uint8_t ui8_error);
void motor_controller_clear_error (void);
uint8_t motor_controller_get_error (void);
void motor_set_pwm_duty_cycle (uint16_t ui16_value); // 0 ... 508
/***************************************************************************************/

#endif /* _MOTOR_H_ */
