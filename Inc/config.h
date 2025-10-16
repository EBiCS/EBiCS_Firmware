/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"

// System constants, don't touch!
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);
#define DISPLAY_TYPE_KUNTENG (1<<1)							// For Kunteng display
#define DISPLAY_TYPE_BAFANG_LCD (1<<2)						// For 'Blaupunkt' Display of Prophete Entdecker
#define DISPLAY_TYPE_BAFANG_850_860 (1<<3)					// Bafang 850/860. Can do 9k6 baud, also compatible with 1200 baud detection at startup
#define DISPLAY_TYPE_KINGMETER_618U (1<<4)                  // King-Meter 618U protocol ( J-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<5)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_EBiCS (1<<6)                  			// Protocol using the ANT+ LEV logic
#define DISPLAY_TYPE_NO2 (1<<7)								// For China Protokoll "No_2" S866 display for example
#define DISPLAY_TYPE_BAFANG         (DISPLAY_TYPE_BAFANG_LCD|DISPLAY_TYPE_BAFANG_850_860)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define EXTERNAL 1
#define INTERNAL 0
//----------------------------------------------------------------------
// advanced setting, don't touch, if you don't know what you are doing!
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
//#define FAST_LOOP_LOG
//#define DISABLE_DYNAMIC_ADC
//#define INDIVIDUAL_MODES
//#define SPEEDTHROTTLE
#define SIXSTEPTHRESHOLD 20000
#define SPEED_PLL 0 //1 for using PLL, 0 for angle extrapolation
#define P_FACTOR_PLL 10
#define I_FACTOR_PLL 10

//----------------------------------------------------------------------
//Battery bar settings for Kunteng and Bafang Display

//#define BATTERY_LEVEL_0 320000 //moved to display_kunteng.h
#define BATTERY_LEVEL_1 323000
#define BATTERY_LEVEL_2 329000
#define BATTERY_LEVEL_3 344000
#define BATTERY_LEVEL_4 368000
#define BATTERY_LEVEL_5 380000

//----------------------------------------------------------------------
//PI-control factor settings
#define P_FACTOR_I_Q 50
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 50
#define I_FACTOR_I_D 2
#define P_FACTOR_SPEED 1
#define I_FACTOR_SPEED 10

//----------------------------------------------------------------------
//PAS mode settings
//#define DIRDET
#define FRAC_HIGH 30
#define FRAC_LOW 15
#define PAS_TIMEOUT 3000
#define RAMP_END 1200
#define PAS_IMP_PER_TURN 32

//---------------------------------------------------------------------
//Throttle settings
#define THROTTLE_OFFSET 1250   //only default value, throttle offset is set at startup automatically
#define THROTTLE_MAX 2850
#define THROTTLE_OVERRIDE

//--------------------------------------------------------------------
//Speed settings
#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 11 //11 for BionX IGH3
#define SPEEDLIMIT 25
#define PULSES_PER_REVOLUTION 1
#define SPEEDSOURCE INTERNAL
#define SPEEDFILTER 1
#define SPDSHFT 0

//---------------------------------------------------------------------
//power settings
#define PH_CURRENT_MAX 1200
#define BATTERYCURRENT_MAX 14000
#define REVERSE 1 //1 for normal direction, -1 for reverse
#define PUSHASSIST_CURRENT 300
#define VOLTAGE_MIN 1320 //33V

//---------------------------------------------------------------------
//torquesensor settings
#define TS_COEF 2400
#define TS_MODE
//#define TQONAD1

//---------------------------------------------------------------------
//Display settings
#define DISPLAY_TYPE DISPLAY_TYPE_DEBUG

//---------------------------------------------------------------------
//Regen settings

#define REGEN_CURRENT 800
#define REGEN_CURRENT_MAX 10000
//#define ADC_BRAKE

//---------------------------------------------------------------------
#define AUTODETECT 0

#endif /* CONFIG_H_ */
