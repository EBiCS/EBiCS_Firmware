/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "main.h"

#define KEY 0xca

#define EEPROM_BASE_ADDRESS 			0x4000
#define ADDRESS_KEY 				EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL 			1 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_CHARACTARISTIC 		2 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_SIZE	 		3 + EEPROM_BASE_ADDRESS
#define ADDRESS_MAX_SPEED	 		4 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_CONTROL_MODE 	5 + EEPROM_BASE_ADDRESS
#define ADDRESS_CONTROLLER_MAX_CURRENT		6 + EEPROM_BASE_ADDRESS

void eeprom_init (void);
void eeprom_write_if_values_changed (void);

#endif /* _EEPROM_H_ */
