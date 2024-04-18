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
#define DISPLAY_TYPE_M365DASHBOARD (1<<1)
#define DISPLAY_TYPE_DEBUG (1<<0)
#define EXTERNAL 1
#define INTERNAL 0
//----------------------------------------------------------------------
// advanced setting, don't touch, if you don't know what you are doing!
#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028
#define CAL_BAT_V 25
#define CAL_V 15LL<<8
#define CAL_I 38LL<<8
// BionX IGH3
//#define INDUCTANCE 12LL
//#define RESISTANCE 220LL
//#define FLUX_LINKAGE 2400LL
//#define GAMMA 13LL

// Hoverboard Motor
#define INDUCTANCE 11LL
#define RESISTANCE 120LL
#define FLUX_LINKAGE 2000LL
#define GAMMA 13LL


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

//---------------------------------------------------------------------
//Throttle settings
#define THROTTLE_OFFSET 690   //only default value, throttle offset is set at startup automatically
#define THROTTLE_MAX 2850
#define THROTTLE_OVERRIDE
#define THROTTLEOFFSET 45
#define THROTTLEMAX 200
#define BRAKEOFFSET 38
#define BRAKEMAX 205

//--------------------------------------------------------------------
//Speed settings
#define WHEEL_CIRCUMFERENCE 710
#define GEAR_RATIO 22 //11 for BionX IGH3
#define SPEEDLIMIT 25
#define PULSES_PER_REVOLUTION 1
#define SPEEDSOURCE INTERNAL
#define SPEEDFILTER 1
#define SPDSHFT 0

//---------------------------------------------------------------------
//power settings
#define PH_CURRENT_MAX 1200
#define BATTERYCURRENT_MAX 14000
#define REVERSE -1 //1 for normal direction, -1 for reverse
#define PUSHASSIST_CURRENT 300
#define VOLTAGE_MIN 1320 //33V

// motor current limits for invividual modes in mA, see default settings at https://max.cfw.sh/#
#define PH_CURRENT_MAX_ECO 300
#define PH_CURRENT_MAX_NORMAL 900
#define PH_CURRENT_MAX_SPORT 1200

// speed limits for invividual modes in kph
#define SPEEDLIMIT_ECO 6
#define SPEEDLIMIT_NORMAL 20
#define SPEEDLIMIT_SPORT 50

// battery voltage limits in mV
#define BATTERYVOLTAGE_MAX 41500
#define BATTERYVOLTAGE_MIN 33000

//---------------------------------------------------------------------
//torquesensor settings
#define TS_COEF 2400
#define TS_MODE
//#define TQONAD1

//---------------------------------------------------------------------
//Display settings
#define DISPLAY_TYPE DISPLAY_TYPE_M365DASHBOARD
#define BATTERY_COMMUNICATION

//---------------------------------------------------------------------
//Regen settings

#define REGEN_CURRENT 800
#define REGEN_CURRENT_MAX 10000
//#define ADC_BRAKE

//---------------------------------------------------------------------
#define AUTODETECT 0

#endif /* CONFIG_H_ */
