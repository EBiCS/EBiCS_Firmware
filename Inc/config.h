//place all your personal configurations here and keep this file when updating!
#ifndef CONFIG_H
#define CONFIG_H
#include "stdint.h"

#define DISPLAY_TYPE_KINGMETER_618U (1<<4)                  // King-Meter 618U protocol (KM5s, EBS-LCD2, J-LCD, SW-LCD)
#define DISPLAY_TYPE_KINGMETER_901U (1<<8)                  // King-Meter 901U protocol (KM5s)
#define DISPLAY_TYPE_KINGMETER      (DISPLAY_TYPE_KINGMETER_618U|DISPLAY_TYPE_KINGMETER_901U)
#define DISPLAY_TYPE DISPLAY_TYPE_KINGMETER_618U

#define wheel_circumference 2.202 //wheel circumference in m

#endif
