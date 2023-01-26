/*
 * config.h
 *
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
#define P_FACTOR_I_Q 50
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 50
#define I_FACTOR_I_D 2
#define P_FACTOR_SPEED 1
#define I_FACTOR_SPEED 10
#define TS_COEF 16000000
#define PAS_TIMEOUT 8000
#define RAMP_END 1200
#define THROTTLE_OFFSET 700
#define THROTTLE_MAX 2850
#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 11 //11 for BionX IGH3
#define SPEEDLIMIT 35
#define PULSES_PER_REVOLUTION 1
#define PH_CURRENT_MAX 600
#define BATTERYCURRENT_MAX 15000
#define SPEC_ANGLE -167026406L //Value no longer needed
//#define DIRDET
#define FRAC_HIGH 30
#define FRAC_LOW 15
//#define TS_MODE
//#define TQONAD1
#define DISPLAY_TYPE DISPLAY_TYPE_KINGMETER_901U
#define REVERSE 1
#define PUSHASSIST_CURRENT 300
#define VOLTAGE_MIN 300
#define REGEN_CURRENT 200
//#define FAST_LOOP_LOG
//#define DISABLE_DYNAMIC_ADC
//#define INDIVIDUAL_MODES
//#define SPEEDTHROTTLE
#define THROTTLE_OVERRIDE
#define REGEN_CURRENT_MAX 10000


#define P_FACTOR_PLL 10
#define I_FACTOR_PLL 10
#define AUTODETECT 0
#define SPEED_PLL 0 //1 for using PLL, 0 for angle extrapolation

#define EXTERNAL 1
#define INTERNAL 0
#define SPEEDSOURCE EXTERNAL
#define SPEEDFILTER 1
#define SIXSTEPTHRESHOLD 20000

#define SPDSHFT 0
//#define ADC_BRAKE

#endif /* CONFIG_H_ */
