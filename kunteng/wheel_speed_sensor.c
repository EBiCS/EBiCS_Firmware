/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_it.h"
#include "gpio.h"

void wheel_speed_sensor_init (void)
{
  //whell speed sensor pin as input
  GPIO_Init(WHEEL_SPEED_SENSOR__PORT,
	    WHEEL_SPEED_SENSOR__PIN,
	    GPIO_MODE_IN_PU_NO_IT); // input pull-up, no external interrupt
}
