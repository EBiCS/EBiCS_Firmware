/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"
#define DISPLAY_TYPE_KINGMETER_618U (1<<4)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<8)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define DISPLAY_TYPE_BAFANG (1<<2)							// For 'Blaupunkt' Display of Prophete Entdecker
#define DISPLAY_TYPE_KUNTENG (1<<1)							// For ASCII-Output in Debug mode
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);

#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028
#define CAL_BAT_V 256
#define CAL_V 15LL<<8
#define CAL_I 38LL<<8
#define INDUCTANCE 6LL
#define RESISTANCE 40LL
#define FLUX_LINKAGE 1200LL
#define GAMMA 9LL
#define BATTERY_LEVEL_1 323000
#define BATTERY_LEVEL_2 329000
#define BATTERY_LEVEL_3 344000
#define BATTERY_LEVEL_4 368000
#define BATTERY_LEVEL_5 380000
#define P_FACTOR_I_Q 0.1L
#define I_FACTOR_I_Q 0.01L
#define P_FACTOR_I_D 1
#define I_FACTOR_I_D 1
#define TS_COEF 60000
#define PAS_TIMEOUT 8000
#define RAMP_END 1600
#define THROTTLE_OFFSET 1255
#define THROTTLE_MAX 4096
#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 60
#define SPEEDLIMIT 25
#define PULSES_PER_REVOLUTION 1
#define PH_CURRENT_MAX 300
#define SPEC_ANGLE -715827882LL
#define TS_MODE
#define DISPLAY_TYPE DISPLAY_TYPE_DEBUG //ASCII Printout for debugging

#endif /* CONFIG_H_ */
