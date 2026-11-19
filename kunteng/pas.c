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

void pas_init (void)
{
  //PAS pin as external input pin
  GPIO_Init(PAS__PORT,
	    PAS__PIN,
	    GPIO_MODE_IN_PU_NO_IT); // input pull-up, no external interrupt
}
