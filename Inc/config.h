//place all your personal configurations here and keep this file when updating!
#ifndef CONFIG_H
#define CONFIG_H
#include "stdint.h"

#define DISPLAY_TYPE_KINGMETER_618U (1<<4)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<8)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER_FISCHER_1822 (1<<16)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U|DISPLAY_TYPE_KINGMETER_FISCHER_1822)
#define DEBUG_SLOW_LOOP (1<<2)
#define DEBUG_FAST_LOOP (1<<3)

#define DISPLAY_TYPE DEBUG_SLOW_LOOP

//#define DISPLAY_TYPE DEBUG_SLOW_LOOP
//#define DISPLAY_TYPE DEBUG_FAST_LOOP


#define wheel_circumference 2.202 	//wheel circumference in m
#define THROTTLE_OFFSET 1275//950//1210 			//ADC-value at closed throttle 670 for throttle, 1255 for TS
#define THROTTLE_MAX 3015
#define TS_COEF 800					//coefficient for torque-sensor-mode
//#define TS_MODE						//Torquesensor-Mode
#define PAS_TIMEOUT 12000			//time tics @ 16kHz untils motor stops
#define RAMP_END 4000					//time tics @ 16kHz where motor reaches full level power
#define PH_CURRENT_MAX 300			//iq value (phase current in rotating frame), not calibrated yet

#define P_FACTOR_I_Q 0.01				//proportional factor for PI control of iq
#define I_FACTOR_I_Q 0.008			//integral factor for PI control of iq
#define P_FACTOR_I_D 1L				//proportional factor for PI control of id
#define I_FACTOR_I_D 1L				//integral factor for PI control of id


//#define SPEC_ANGLE  -811271600L		//Shengyi Mittelmotor per trial and error
#define SPEC_ANGLE  -1312351118L		//Shengyi Heckmotor aus Fischer ETH1606 per trial and error
#define FILTER_DELAY 59652323<<4	 //1073741824L	// for angle correction of i_alfa + i_beta

#define OFFSET_A 1993 //1025 				//Offset of current sensing phase A
#define OFFSET_B 1977 //1022				//Offset of current sensing phase B
#define OFFSET_C 1966 //1042				//Offset of current sensing phase C

//LeftShift um bei R*L in vern�nftigen Ganzzahlbereich zu kommen
#define CAL_V 15LL<<8			  		// ADC*25,6 mV/Digit*1/SQRT(3)=15,  bei �bergabe an Observerfunktion >>11 um Dutycycle reinzurechnen.  Im Oberserver kommt Spannung in mV *2^-8 an.
#define CAL_I 38LL<<8					// ADC * 37,5 mA/Digit. Strom kommt in mA *2^-8 im Observer an. Siehe Post Nr. 99 im Thread (f�r 12 FET)

//Constants for Motor model of observer
#define INDUCTANCE	11LL		//9 Hoverboard,6 BionX,war nach Messung 13 (mit einfachem LCR-Tester)		//H = V*s/A Induktivit�t in �H/100 Shengyi hat 200�Henry Induktivit�t 2^16*0,0002 -->>>16 in Observer um auf Henry zukommen
#define RESISTANCE 120LL		//220 BionX,F�r ShengyiMM 80	//Ohm = V/A Widerstand in Shengi ist 117mOhm -->2^9*0,117 >>9 in Observer um auf Ohm zu kommen.
#define FLUX_LINKAGE 2000LL			//2400 BionX,F�r ShengyiMM 1800| V*s/rad von Hand angepasst
#define GAMMA 13LL					//F�r ShengyiMM 10| per trial and error


#define _T 2048						//Periode des Timers1 zur Einstellung der PWM Frequenz 2048 ergibt 16kHz
#define TRIGGER_OFFSET_ADC 75
#define TRIGGER_DEFAULT 2020
#define TRIGGER_THRESHOLD 1600


#endif
