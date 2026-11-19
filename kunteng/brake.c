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
#include "main.h"
#include "interrupts.h"
#include "brake.h"
#include "ebike_app.h"
#include "motor.h"
#include "pwm.h"
#include "settings.h"

// Brake signal
void EXTI_PORTA_IRQHandler(void) __interrupt(EXTI_PORTA_IRQHANDLER)
{
  if (brake_is_set())
  {
    motor_controller_set_state (MOTOR_CONTROLLER_STATE_BRAKE);
    motor_set_regen_current_max (kt_settings.ui8_regen_current_a * 2); // ebrake/regen with the configured current
    motor_set_pwm_duty_cycle_target (0);
    ebike_app_cruise_control_stop ();

    if (!settings_flag (KT_FLAG_DIRECT_DRIVE))
    {
      TIM1->BKR &= ~TIM1_BKR_MOE; // disable PWM signal only if motor is not direct drive
      motor_set_pwm_duty_cycle (0);
    }
  }
  else
  {
    motor_controller_reset_state (MOTOR_CONTROLLER_STATE_BRAKE);
    motor_set_regen_current_max (0); // disable ebrake/regen
  }
}

void brake_init (void)
{
  //brake pin as external input pin interrupt
  GPIO_Init(BRAKE__PORT,
	    BRAKE__PIN,
	    GPIO_MODE_IN_FL_IT); // with external interrupt

  //initialize the Interrupt sensitivity
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA,
			    EXTI_SENSITIVITY_RISE_FALL);
}

BitStatus brake_is_set (void)
{
  if (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) == 0)
    return 1;
  else
    return 0;
}

