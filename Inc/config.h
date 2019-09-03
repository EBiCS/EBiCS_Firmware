//place all your personal configurations here and keep this file when updating!
#ifndef CONFIG_H
#define CONFIG_H
#include "stdint.h"

#define DISPLAY_TYPE_KINGMETER_618U (1<<4)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<8)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define DISPLAY_TYPE_BAFANG (1>>2)							// For "Blaupunkt" Display of Prophete Entdecker
#define DISPLAY_TYPE_DEBUG (1>>0)							// For ASCII-Output in Debug mode

//#define DISPLAY_TYPE DISPLAY_TYPE_KINGMETER_618U			//Kingmeter J-LCD
//#define DISPLAY_TYPE DISPLAY_TYPE_KINGMETER_901U			//Kingmeter KM5S
//#define DISPLAY_TYPE DISPLAY_TYPE_BAFANG					//Bafang "Blaupunkt"
#define DISPLAY_TYPE DISPLAY_TYPE_DEBUG					//ASCII Parameter Printout on UART

#define DISABLE_DYNAMIC_ADC
#define FAST_LOOP_LOG

#define WHEEL_CIRCUMFERENCE 2202L 		//wheel circumference in mm
#define PULSES_PER_REVOLUTION 6
#define THROTTLE_OFFSET 720 			//ADC-value at closed throttle 670 for throttle, 1255 for TS
#define THROTTLE_MAX 3015 				//ADC-value at closed throttle 670 for throttle, 1255 for TS
#define TS_COEF 2000					//coefficient for torque-sensor-mode
//#define TS_MODE						//Torquesensor-Mode
#define PAS_TIMEOUT 12000				//time tics @ 16kHz untils motor stops
#define RAMP_END 4000					//time tics @ 16kHz where motor reaches full level power
#define PH_CURRENT_MAX 400				//iq value (phase current in rotating frame), not calibrated yet

#define P_FACTOR_I_Q 1L					//proportional factor for PI control of iq
#define I_FACTOR_I_Q 0.001L				//integral factor for PI control of iq
#define P_FACTOR_I_D 1L					//proportional factor for PI control of id
#define I_FACTOR_I_D 1L					//integral factor for PI control of id
#define SPEC_ANGLE -357913855  			//-715827882LL	for Shengyi Middrive, -357913855 for BionX IGH3		//motor specific angle, refer to chapter 8.3.3 of UM1052.shengyi:-715827882L, 715827882 536870912 357913941L; //357913941 298261617 119304647L // 30° BionX IGH3 motor specific angle, refer to chapter 8.8.3 of UM1052 180° maped to 2^31

#define _T 2028
#define OFFSET_A 1980 //1025 				//Offset of current sensing phase A
#define OFFSET_B 1932 //1022				//Offset of current sensing phase B
#define OFFSET_C 1925 //1042				//Offset of current sensing phase C

#define CAL_BAT_V 256LL

//LeftShift um bei R*L in vernünftigen Ganzzahlbereich zu kommen
#define CAL_V 15LL<<8			  		// ADC*25,6 mV/Digit*1/SQRT(3)=15,  bei Übergabe an Observerfunktion >>11 um Dutycycle reinzurechnen.  Im Oberserver kommt Spannung in mV *2^-8 an.
#define CAL_I 38LL<<8					// ADC * 37,5 mA/Digit. Strom kommt in mA *2^-8 im Observer an. Siehe Post Nr. 99 im Thread (für 12 FET)

//Constants for Motor model of observer
#define INDUCTANCE	6LL		//war nach Messung 13 (mit einfachem LCR-Tester)		//H = V*s/A Induktivität in µH/100 Shengyi hat 200µHenry Induktivität 2^16*0,0002 -->>>16 in Observer um auf Henry zukommen
#define RESISTANCE 40LL		//war nach Messung 60	//Ohm = V/A Widerstand in Shengi ist 117mOhm -->2^9*0,117 >>9 in Observer um auf Ohm zu kommen.
#define FLUX_LINKAGE 1200LL			//V*s/rad von Hand angepasst
#define GAMMA 9LL					//per trial and error

#define BATTERY_LEVEL_1 323000        //Voltage in mV*10
#define BATTERY_LEVEL_2 329000
#define BATTERY_LEVEL_3 344000
#define BATTERY_LEVEL_4 368000
#define BATTERY_LEVEL_5 380000
#endif
