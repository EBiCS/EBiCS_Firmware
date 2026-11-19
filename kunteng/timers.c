/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "stm8s.h"
#include "stm8s_tim2.h"

void timer2_init (void)
{
  uint16_t ui16_i;

  // TIM2 Peripheral Configuration
  TIM2_DeInit();
  TIM2_TimeBaseInit(TIM2_PRESCALER_2048, 0xffff); // each incremment at every 128us
  TIM2_Cmd(ENABLE); // TIM2 counter enable

  // IMPORTANT: this software delay is needed so timer2 work after this
  for(ui16_i = 0; ui16_i < (29000); ui16_i++) { ; }
}
