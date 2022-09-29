/*
 * config.h
 
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"
#define DISPLAY_TYPE_EBiCS (1<<5)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_618U (1<<3)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<4)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define DISPLAY_TYPE_BAFANG (1<<2)							// For 'Blaupunkt' Display of Prophete Entdecker
#define DISPLAY_TYPE_KUNTENG (1<<1)							// For ASCII-Output in Debug mode
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);

#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028
#define CAL_BAT_V 256
#define CAL_V 25
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
#define P_FACTOR_I_Q 500
#define I_FACTOR_I_Q 20
#define P_FACTOR_I_D 500
#define I_FACTOR_I_D 20
#define P_FACTOR_SPEED 1
#define I_FACTOR_SPEED 10
#define TS_COEF 8000
#define PAS_TIMEOUT 8000
#define RAMP_END 1600
#define THROTTLE_OFFSET 2050
#define THROTTLE_MAX 800
#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 11 //dummy for testing
#define SPEEDLIMIT 40
#define PULSES_PER_REVOLUTION 1
#define PH_CURRENT_MAX 1200
#define BATTERYCURRENT_MAX 15000
#define SPEC_ANGLE 0//-167026406L //BionX IGH3 -143165476
//#define DIRDET
#define FRAC_HIGH 30
#define FRAC_LOW 15
//#define TS_MODE
//#define TQONAD1
#define DISPLAY_TYPE DISPLAY_TYPE_KINGMETER_901U
#define REVERSE 1
#define PUSHASSIST_CURRENT 150
#define VOLTAGE_MIN 300
#define BATTERYVOLTAGE_MAX 41500
#define REGEN_CURRENT 200
//#define FAST_LOOP_LOG
//#define DISABLE_DYNAMIC_ADC
//#define INDIVIDUAL_MODES
//#define SPEEDTHROTTLE
//#define THROTTLE_OVERRIDE
#define REGEN_CURRENT_MAX 10000
#define SPEED_PLL
#define P_FACTOR_PLL 10
#define I_FACTOR_PLL 10
#define AUTODETECT 0

#define EXTERNAL 1
#define INTERNAL 0
#define SPEEDSOURCE EXTERNAL
#define SPEEDFILTER 1
#define SIXSTEPTHRESHOLD 20000

#define SPDSHFT 0
//#define ADC_BRAKE

#endif /* CONFIG_H_ */
