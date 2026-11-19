/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

//#define DEBUG_UART

#define CONTROLLER_TYPE_S06S 1
#define CONTROLLER_TYPE_S12S 2
#ifndef CONTROLLER_TYPE // settings can be passed with -D by the EBiCS Setup app (kunteng/build.py)
#define CONTROLLER_TYPE CONTROLLER_TYPE_S06S
#endif

#define MOTOR_TYPE_Q85 1
#define MOTOR_TYPE_Q100 2
#define MOTOR_TYPE_Q11 3
#define MOTOR_TYPE_EUC2 4

#ifndef MOTOR_TYPE
#define MOTOR_TYPE MOTOR_TYPE_Q85
#endif

#if MOTOR_TYPE == MOTOR_TYPE_Q11
#define MOTOR_TYPE_DIRECT_DRIVE
#endif

#if MOTOR_TYPE == MOTOR_TYPE_Q11
#undef MOTOR_TYPE
#define MOTOR_TYPE MOTOR_TYPE_Q85
#endif

// *************************************************************************** //
// Configure throotle / PAS / torque sensor control
// Configure PWM duty_cycle control or motor current/torque + wheel speed control

// choose between throotle and/or pas or torque sensor
#ifndef EBIKE_THROTTLE_TYPE
#define EBIKE_THROTTLE_TYPE	EBIKE_THROTTLE_TYPE_THROTTLE_PAS
//#define EBIKE_THROTTLE_TYPE	EBIKE_THROTTLE_TYPE_TORQUE_SENSOR
#endif

// next, choose one of the both
//#define EBIKE_THROTTLE_TYPE_THROTTLE_PAS_PWM_DUTY_CYCLE // simple PWM duty_cycle control (is needed for development and debug)
#define EBIKE_THROTTLE_TYPE_THROTTLE_PAS_CURRENT_SPEED // control using motor current/torque and wheel speed

// next, if enabled, output of torque sensor algorithm is the human power (torque * cadence) other way will be the same as the torque signal
#define EBIKE_THROTTLE_TYPE_TORQUE_SENSOR_HUMAN_POWER // output of torque sensor algorithm is the human power (torque * cadence)


#define EBIKE_THROTTLE_TYPE_THROTTLE_PAS		1
#define EBIKE_THROTTLE_TYPE_TORQUE_SENSOR		2
// *************************************************************************** //


// MOTOR_TYPE_EUC2
// 28V --> 66 ERPs
// 0.424 for each ERPs
#if (MOTOR_TYPE == MOTOR_TYPE_Q85) || (MOTOR_TYPE == MOTOR_TYPE_Q100)
#define MOTOR_KVOLTS_PER_ERPS 73 //Q85 motor
#elif MOTOR_TYPE == MOTOR_TYPE_EUC2
#define MOTOR_KVOLTS_PER_ERPS 107 //0.424 << 8 EUC2 motor
#endif

#if defined (MOTOR_CURRENT_MAX_A) // from the setup app: amps -> ADC steps (0.5 A / 0.125 A)
#define ADC_MOTOR_CURRENT_MAX_MED_10B 10144
#define ADC_MOTOR_CURRENT_MAX (MOTOR_CURRENT_MAX_A * 2)
#define ADC_MOTOR_CURRENT_MAX_10B (MOTOR_CURRENT_MAX_A * 8)
#ifndef MOTOR_REGEN_CURRENT_MAX_A
#define MOTOR_REGEN_CURRENT_MAX_A 0
#endif
#define ADC_MOTOR_REGEN_CURRENT_MAX (MOTOR_REGEN_CURRENT_MAX_A * 2)
#elif CONTROLLER_TYPE == CONTROLLER_TYPE_S06S
#define ADC_MOTOR_CURRENT_MAX_MED_10B 10144 // ADC_MOTOR_CURRENT_MAX_ZERO_VALUE_10B << 5
#define ADC_MOTOR_CURRENT_MAX 30 // 30 = 15A; 1 --> 0.5A
#define ADC_MOTOR_CURRENT_MAX_10B 120 // 120 = 15A; 1 --> 0.125A
//#define ADC_MOTOR_REGEN_CURRENT_MAX 30 	// 15A but the brake/regen must be only for a ferw seconds other way will be a problem!!
//#define ADC_MOTOR_REGEN_CURRENT_MAX 1 	// 15A but the brake/regen must be only for a ferw seconds other way will be a problem!!
#define ADC_MOTOR_REGEN_CURRENT_MAX 1 	// 15A but the brake/regen must be only for a ferw seconds other way will be a problem!!
#elif CONTROLLER_TYPE == CONTROLLER_TYPE_S12S
#define ADC_MOTOR_CURRENT_MAX_MED_10B 10144 // ADC_MOTOR_CURRENT_MAX_ZERO_VALUE_10B << 5
#define ADC_MOTOR_CURRENT_MAX 50 // 50 = 25A; 1 --> 0.5A
#define ADC_MOTOR_CURRENT_MAX_10B 200 // 200 = 25A; 1 --> 0.125A
#define ADC_MOTOR_REGEN_CURRENT_MAX 30 // 15A;  1 --> 0.5A
#endif

#if CONTROLLER_TYPE == CONTROLLER_TYPE_S06S
//#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP 100
//#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP 30
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP 25
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP 10
#elif CONTROLLER_TYPE == CONTROLLER_TYPE_S12S
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP 50
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP 10
#endif

// Phase current: max of +-15.5 amps
// 512 --> 15.5 amps
// 1 ADC increment --> 0.030A
// RMS value --> max value * 0.707
#define ADC_PHASE_B_CURRENT_FACTOR 333 // 0.1 / 0.030A * 0.707 = 3.3

#if (MOTOR_TYPE == MOTOR_TYPE_Q85) || (MOTOR_TYPE == MOTOR_TYPE_Q100)
// This value must be found experimenting. Motor should rotate forward and have a good torque,
// a value to much higher or lower will make the motor not having torque while the motor starting up.
// This value can be tested with motor blocked, at startup, to found a value where is does have the best torque at startup
// A value of MOTOR_ROTOR_OFFSET_ANGLE = 202 was found to be a good one for BMSBattery Q85 motor with S06S controller
#ifndef MOTOR_ROTOR_OFFSET_ANGLE
#define MOTOR_ROTOR_OFFSET_ANGLE 202
#endif

// This value at 127 is the default but a value a bit higher or lower my improve the controller efficiency
// This value can be tested with motor running at medium speed, where it is already running with sinewave interpolation (not at startup)
// A way to test: put the motor with the same mechanical load (like on a bike training roller) and with a constant speed
// changing this value will make the motor to need more or less current to mantain the same speed -- adjust to use the least current possible
// A value of FOC_READ_ID_CURRENT_ANGLE_ADJUST = 137 was found to be a good one for BMSBattery Q85 motor with S06S controller
#ifndef FOC_READ_ID_CURRENT_ANGLE_ADJUST
#define FOC_READ_ID_CURRENT_ANGLE_ADJUST 137
#endif

// This value is ERPS speed after which a transition happens from sinewave no interpolation to have
// interpolation 60 degrees and must be found experimentally but a value of 40 may be good
#ifndef MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES
#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES 15
#endif

// For some motors with not very well placed mosfets at 120 degrees between each of them. May be easier to keep disable this option
//#define DO_SINEWAVE_INTERPOLATION_360_DEGREES
#ifdef DO_SINEWAVE_INTERPOLATION_360_DEGREES
// This value is ERPS speed after which a transition happens from sinewave 60 degrees to have
// interpolation 360 degrees and must be found experimentally but a value of 100 may be good
#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_360_DEGREES 100
#endif
#elif MOTOR_TYPE == MOTOR_TYPE_EUC2
#endif

#if MOTOR_TYPE == MOTOR_TYPE_Q85
#define PWM_CYCLES_COUNTER_MAX 3125 // 5 erps minimum speed; 1/5 = 200ms; 200ms/64us = 3125
#elif MOTOR_TYPE == MOTOR_TYPE_Q100
#define PWM_CYCLES_COUNTER_MAX 1041 // 15 erps minimum speed
#endif


#define PWM_CYCLES_SECOND 15625L // 1 / 64us(PWM period)

#define SPEED_INVERSE_INTERPOLATION 625 // experimental value; min speed aftwer which interpolation starts

#define PWM_DUTY_CYCLE_MAX 254
#define PWM_DUTY_CYCLE_MIN 20
#define MIDDLE_PWM_DUTY_CYCLE_MAX (PWM_DUTY_CYCLE_MAX/2)

#define ANGLE_1 	(0 + MOTOR_ROTOR_OFFSET_ANGLE)
#define ANGLE_60 	(42 + MOTOR_ROTOR_OFFSET_ANGLE)
#define ANGLE_120 	(85 + MOTOR_ROTOR_OFFSET_ANGLE)
#define ANGLE_180 	(127 + MOTOR_ROTOR_OFFSET_ANGLE)
#define ANGLE_240 	(170 + MOTOR_ROTOR_OFFSET_ANGLE)
#define ANGLE_300 	(212 + MOTOR_ROTOR_OFFSET_ANGLE)
#define ANGLE_360 	(255 + MOTOR_ROTOR_OFFSET_ANGLE)

// angle offset to make the FOC read Id current at 127 as default, just to be easier for the user when customizing
#define FOC_READ_ID_CURRENT_OFFSET (127 - ((uint8_t) ANGLE_180))

#define MOTOR_OVER_SPEED_ERPS 520 // motor max speed, protection max value | 30 points for the sinewave at max speed

#if CONTROLLER_TYPE == CONTROLLER_TYPE_S06S
#define MOTOR_SPEED_CONTROLLER_KP 2 // x << 5
#elif CONTROLLER_TYPE == CONTROLLER_TYPE_S12S
#define MOTOR_SPEED_CONTROLLER_KP 1 // x << 5
#endif
#define MOTOR_SPEED_CONTROLLER_OUTPUT_MAX 8160 // PWM max duty_cycle << 5

#define MOTOR_CURRENT_CONTROLLER_KP 10
#define MOTOR_CURRENT_CONTROLLER_OUTPUT_MAX 16320

#define MOTOR_PWM_TICKS_PER_MS 16

//TORQUE SENSOR
#define ADC_THROTTLE_MIN_VALUE 51 // 1V = 51
//#define ADC_THROTTLE_MAX_VALUE 229 // 4.5V = 229
#define ADC_THROTTLE_MAX_VALUE 183 // 4.5V = 229
//#define ADC_THROTTLE_MIN_VALUE 45
//#define ADC_THROTTLE_MAX_VALUE 182
#define ADC_THROTTLE_MIN_ERROR 35
#define ADC_THROTTLE_MAX_ERROR 192
#define CRUISE_CONTROL_MIN 20

// *************************************************************************** //
// PAS
#ifndef PAS_NUMBER_MAGNETS
#define PAS_NUMBER_MAGNETS 8
#endif
#ifndef PAS_MAX_CADENCE_RPM
#define PAS_MAX_CADENCE_RPM 90
#endif
#ifndef PAS_DIRECTION
#define PAS_DIRECTION PAS_DIRECTION_RIGHT
#endif

#define PAS_ASSIST_LEVEL_0 0.0
#ifndef PAS_ASSIST_LEVEL_1
#define PAS_ASSIST_LEVEL_1 0.2
#endif
#ifndef PAS_ASSIST_LEVEL_2
#define PAS_ASSIST_LEVEL_2 0.4
#endif
#ifndef PAS_ASSIST_LEVEL_3
#define PAS_ASSIST_LEVEL_3 0.6
#endif
#ifndef PAS_ASSIST_LEVEL_4
#define PAS_ASSIST_LEVEL_4 0.8
#endif
#ifndef PAS_ASSIST_LEVEL_5
#define PAS_ASSIST_LEVEL_5 1.0
#endif


#define PAS_DIRECTION_RIGHT 0
#define PAS_DIRECTION_LEFT 1

// (1/(150RPM/60)) / (PAS_NUMBER_MAGNETS * 0.000064)
#define PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS  (6250 / PAS_NUMBER_MAGNETS) // max hard limit to 150RPM PAS cadence
#define PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS  (156250 / PAS_NUMBER_MAGNETS) // min hard limit to 6RPM PAS cadence
// *************************************************************************** //

// *************************************************************************** //
// Wheel speed sensor
#define WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS  135 // something like 200m/h with a 6'' wheel
#define WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS  64000 // just a bit less than 2^16 (uint16_t)
// *************************************************************************** //

// *************************************************************************** //
// EEPROM memory variables default values

#define DEFAULT_VALUE_ASSIST_LEVEL 		2
#define DEFAULT_VALUE_MOTOR_CHARACTARISTIC 	202 // for Q85 motor (12.6 * 16)
#ifndef DEFAULT_VALUE_WHEEL_SIZE
#define DEFAULT_VALUE_WHEEL_SIZE	 	20 // 26''
#endif
#ifndef DEFAULT_VALUE_MAX_SPEED
#define DEFAULT_VALUE_MAX_SPEED	 		25
#endif
#define DEFAULT_VALUE_POWER_ASSIST_CONTROL_MODE 1
#define DEFAULT_VALUE_CONTROLLER_MAX_CURRENT	10

// *************************************************************************** //
// BATTERY

#ifndef BATTERY_LI_ION_CELLS_NUMBER
#define BATTERY_LI_ION_CELLS_NUMBER 7 // 7S = 24V battery pack
#endif
#if (BATTERY_LI_ION_CELLS_NUMBER < 7) || (BATTERY_LI_ION_CELLS_NUMBER > 16)
#error "BATTERY_LI_ION_CELLS_NUMBER must be 7 ... 16 (the battery voltage ADC ends at about 69 V)"
#endif

// ADC Battery voltage: 1 ADC step (8 bits) = 0.272 volts
#define ADC_BATTERY_VOLTAGE_K 73 // 0.272 << 8

// computed for any cell count (was a table for 7S/10S/13S only): 4.2 V and 3.0 V per cell
#define ADC_BATTERY_VOLTAGE_MAX ((uint8_t) ((BATTERY_LI_ION_CELLS_NUMBER * 4200L) / 272)) // 7S: 108
#define ADC_BATTERY_VOLTAGE_MIN ((uint8_t) ((BATTERY_LI_ION_CELLS_NUMBER * 3000L) / 272)) // 7S: 77
#define ADC_BATTERY_VOLTAGE_MED ((uint16_t) (((ADC_BATTERY_VOLTAGE_MAX + ADC_BATTERY_VOLTAGE_MIN) / 2) << 6))
// nominal voltage the KT display expects (24, 36 or 48)
#if BATTERY_LI_ION_CELLS_NUMBER <= 8
#define COMMUNICATIONS_BATTERY_VOLTAGE 24
#elif BATTERY_LI_ION_CELLS_NUMBER <= 11
#define COMMUNICATIONS_BATTERY_VOLTAGE 36
#else
#define COMMUNICATIONS_BATTERY_VOLTAGE 48
#endif

//#define BATTERY_OVER_VOLTAGE_PROTECTION

// Considering the follow voltage values for each li-ion battery cell
// State of charge 		| voltage
#define LI_ION_CELL_VOLTS_MAX 	4.2
#define LI_ION_CELL_VOLTS_100 	4.05
#define LI_ION_CELL_VOLTS_80 	3.93
#define LI_ION_CELL_VOLTS_60 	3.78
#define LI_ION_CELL_VOLTS_40 	3.60
#define LI_ION_CELL_VOLTS_20 	3.38
#define LI_ION_CELL_VOLTS_0 	3.20
#define LI_ION_CELL_VOLTS_MIN 	3.0

#define BATTERY_PACK_VOLTS_100	(LI_ION_CELL_VOLTS_100 * BATTERY_LI_ION_CELLS_NUMBER) * 256
#define BATTERY_PACK_VOLTS_80 	(LI_ION_CELL_VOLTS_80 * BATTERY_LI_ION_CELLS_NUMBER) * 256
#define BATTERY_PACK_VOLTS_60	(LI_ION_CELL_VOLTS_60 * BATTERY_LI_ION_CELLS_NUMBER) * 256
#define BATTERY_PACK_VOLTS_40	(LI_ION_CELL_VOLTS_40 * BATTERY_LI_ION_CELLS_NUMBER) * 256
#define BATTERY_PACK_VOLTS_20	(LI_ION_CELL_VOLTS_20 * BATTERY_LI_ION_CELLS_NUMBER) * 256
#define BATTERY_PACK_VOLTS_0	(LI_ION_CELL_VOLTS_0 * BATTERY_LI_ION_CELLS_NUMBER) * 256

#endif
