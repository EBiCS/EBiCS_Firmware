//place all your personal configurations here and keep this file when updating!
#ifndef CONFIG_H
#define CONFIG_H
#include "stdint.h"

#define DISPLAY_TYPE_KINGMETER_618U (1<<4)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<8)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define DISPLAY_TYPE DISPLAY_TYPE_KINGMETER_618U

#define wheel_circumference 2.202 	//wheel circumference in m
#define THROTTE_OFFSET 670 			//ACD-value at closed throttle
#define PAS_TIMEOUT 3200			//time tics @ 16kHz untils motor stops
#define RAMP_END 400					//time tics @ 16kHz where motor reaches full level power
#define PH_CURRENT_MAX 250			//iq value (phase current in rotating frame), not calibrated yet

#define P_FACTOR_I_Q 1L				//proportional factor for PI control of iq
#define I_FACTOR_I_Q 0.05L			//integral factor for PI control of iq
#define P_FACTOR_I_D 1L				//proportional factor for PI control of id
#define I_FACTOR_I_D 1L				//integral factor for PI control of id
#define SPEC_ANGLE -715827882L		//motor specific angle, refer to chapter 8.3.3 of UM1052. 715827882 536870912 357913941L; //357913941 298261617 119304647L // 30° BionX IGH3 motor specific angle, refer to chapter 8.8.3 of UM1052 180° maped to 2^31



#endif
