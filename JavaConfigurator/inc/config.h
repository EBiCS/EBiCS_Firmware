/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"
#define DISPLAY_TYPE_EBiCS (1<<5)  
#define DISPLAY_TYPE_KINGMETER_618U (1<<4)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<8)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define DISPLAY_TYPE_BAFANG (1<<2)							// For 'Blaupunkt' Display of Prophete Entdecker
#define DISPLAY_TYPE_KUNTENG (1<<1)							// For ASCII-Output in Debug mode
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);
#define EXTERNAL 1
#define INTERNAL 0

#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028
#define CAL_BAT_V 256
#define CAL_V 15LL<<8
#define CAL_I 38LL<<8
#define INDUCTANCE 6
#define RESISTANCE 40
#define FLUX_LINKAGE 1200
#define GAMMA 9
#define BATTERY_LEVEL_1 323000
#define BATTERY_LEVEL_2 329000
#define BATTERY_LEVEL_3 344000
#define BATTERY_LEVEL_4 368000
#define BATTERY_LEVEL_5 380000
#define P_FACTOR_I_Q 500
#define I_FACTOR_I_Q 20
#define P_FACTOR_I_D 500
#define I_FACTOR_I_D 20
#define P_FACTOR_PLL 5
#define I_FACTOR_PLL 11
#define P_FACTOR_SPEED 100
#define I_FACTOR_SPEED 10
#define SPDSHFT 0
#define SPEEDFILTER 1
#define SIXSTEPTHRESHOLD 3500
#define TS_COEF 60000
#define PAS_TIMEOUT 8000
#define RAMP_END 1600
#define FRAC_HIGH 30
#define FRAC_LOW 15
#define THROTTLE_OFFSET 750
#define THROTTLE_MAX 2600
#define PUSHASSIST_CURRENT 30
#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 60
#define SPEEDLIMIT 25
#define PULSES_PER_REVOLUTION 1
#define PH_CURRENT_MAX 600
#define BATTERYCURRENT_MAX 15000
#define REGEN_CURRENT 200
#define REGEN_CURRENT_MAX 10000
#define VOLTAGE_MIN 500
#define VOLTAGE_MAX 1600
#define SPEC_ANGLE -715827882
#define TS_MODE
#define TQONAD1
#define DISPLAY_TYPE DISPLAY_TYPE_EBiCS
#define SPEED_PLL
#define ADC_BRAKE
#define SPEEDSOURCE EXTERNAL
#define AUTODETECT 0
#define REVERSE 1
#define THROTTLE_OVERRIDE

#endif /* CONFIG_H_ */
