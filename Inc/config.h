/*
 * config.h *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"

//-------------------------------------- defines for better readability, do not touch!
#define DISPLAY_TYPE_EBiCS (1<<5)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_618U (1<<3)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<4)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define DISPLAY_TYPE_BAFANG (1<<2)							// For 'Blaupunkt' Display of Prophete Entdecker
#define DISPLAY_TYPE_KUNTENG (1<<1)							// For ASCII-Output in Debug mode
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);
#define EXTERNAL 1
#define INTERNAL 0
#define RIDEMODE_PAS (1<<0)
#define RIDEMODE_BB_TORQUESENSOR (1<<1)
#define RIDEMODE_KCLAMBER_KASSETTE_SENSOR (1<<2)

//-------------------------------------- advanced settings, do not touch!
#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028
#define CAL_BAT_V 256
#define CAL_V 25
#define CAL_I 30LL<<8
#define INDUCTANCE 6LL
#define RESISTANCE 40LL
#define FLUX_LINKAGE 1200LL
#define GAMMA 9LL
#define SPEC_ANGLE -167026406L //BionX IGH3 -143165476
#define SIXSTEPTHRESHOLD 15000
//#define FAST_LOOP_LOG
//#define DISABLE_DYNAMIC_ADC
//#define INDIVIDUAL_MODES
//#define SPEEDTHROTTLE

//-------------------------------Settings for battery bars on Kunteng display
#define BATTERY_LEVEL_1 323000
#define BATTERY_LEVEL_2 329000
#define BATTERY_LEVEL_3 344000
#define BATTERY_LEVEL_4 368000
#define BATTERY_LEVEL_5 380000

//-------------------------------Settings for current and speed control
#define P_FACTOR_I_Q 50
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 50
#define I_FACTOR_I_D 2
#define P_FACTOR_SPEED 1
#define I_FACTOR_SPEED 10

//-------------------------------Settings for PLL
#define SPEED_PLL 0 //1 for using PLL, 0 for angle extrapolation
#define P_FACTOR_PLL 12
#define I_FACTOR_PLL 12
#define SPDSHFT 0

//------------------------------ Ride mode setting
#define NUMBER_OF_PAS_MAGNETS 12
#define PAS_TIMEOUT 8000
#define RAMP_END 1600
//#define DIRDET
#define FRAC_HIGH 30
#define FRAC_LOW 15
#define TS_COEF 1200 			//12 for Kclamber Sensor
#define RIDEMODE RIDEMODE_BB_TORQUESENSOR
//#define TQONAD1

//------------------------------Throttle settings
#define THROTTLE_OFFSET 900
#define THROTTLE_MAX 2600
//#define THROTTLE_OVERRIDE

//-------------------------------Speed settings
#define WHEEL_CIRCUMFERENCE 2200
#define GEAR_RATIO 11 //dummy for testing
#define SPEEDLIMIT 25
#define PULSES_PER_REVOLUTION 1
#define REVERSE -1
#define SPEEDFILTER 1
#define SPEEDSOURCE EXTERNAL



//----------------------------- Power settings
#define PH_CURRENT_MAX 900
#define BATTERYCURRENT_MAX 15000
#define BATTERYCURRENT_OFFSET 924
//#define ADC_BRAKE
#define REGEN_CURRENT 0
#define REGEN_CURRENT_MAX 10000
#define PUSHASSIST_CURRENT 30
#define VOLTAGE_MIN 1

//----------------------------- Display setting

#define DISPLAY_TYPE DISPLAY_TYPE_DEBUG

//------------------------------Autodetect setting
#define AUTODETECT 0



#endif /* CONFIG_H_ */
