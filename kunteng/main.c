/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "stm8s_itc.h"
#include "stm8s_gpio.h"
#include "interrupts.h"
#include "stm8s_tim2.h"
#include "watchdog.h"
#include "uart.h"
#include "adc.h"
#include "brake.h"
#include "utils.h"
#include "timers.h"
#include "pwm.h"
#include "config.h"
#include "ebike_app.h"
#include "eeprom.h"
#include "motor.h"
#include "pas.h"
#include "wheel_speed_sensor.h"
#include "settings.h"
#include "display.h"

uint16_t ui16_TIM2_counter = 0;
uint16_t ui16_motor_controller_counter = 0;
uint16_t ui16_motor_fast_controller_counter = 0;
uint16_t ui16_ebike_app_controller_counter = 0;
uint16_t ui16_communications_controller_counter = 0;

/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes

// main -- start of firmware and main loop
int main (void);

// With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
// in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
// to place the function prototype in a header file as long as the header file is included in the file that
// contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
// not be in place so the ISR will not be executed when the interrupt occurs.

// Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
// all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// when I have this code inside a function: "static uint8_t ui8_cruise_counter = 0;"
// and the solution was define the variable as global instead
// Another error example:
// *** buffer overflow detected ***: sdcc terminated
// Caught signal 6: SIGABRT

// Brake signal interrupt
void EXTI_PORTA_IRQHandler(void) __interrupt(EXTI_PORTA_IRQHANDLER);

// Motor overcurrent interrupt
void EXTI_PORTD_IRQHandler(void) __interrupt(EXTI_PORTD_IRQHANDLER);

// Timer1/PWM period interrupt
void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM1_UPD_OVF_TRG_BRK_IRQHANDLER);

// UART2 Receive interrupt
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);
// UART2 Transmit interrupt (display messages)
void UART2_TX_IRQHandler(void) __interrupt(UART2_TX_IRQHANDLER);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

int main (void)
{
  //set clock at the max 16MHz
  CLK_HSIPrescalerConfig (CLK_PRESCALER_HSIDIV1);

  settings_init (); // runtime settings from the data EEPROM (written by the EBiCS Setup app)
  gpio_init ();
  brake_init ();
  while (brake_is_set()) ; // hold here while brake is pressed -- this is a protection for development
  debug_pin_init ();
  timer2_init ();
  uart_init ();
  pwm_init_bipolar_4q ();
  hall_sensor_init ();
  adc_init ();
  eeprom_init ();
  motor_init ();
  pas_init ();
  wheel_speed_sensor_init ();
  display_init ();
  enableInterrupts ();

  while (1)
  {
#ifdef DEBUG_UART
    printf ("%d, %d, %d, %d\n", ui16_motor_get_motor_speed_erps (), ui16_duty_cycle, ui8_motor_commutation_type, ui8_angle_correction);
#endif

    // because of continue; at the end of each if code block that will stop the while (1) loop there,
    // the first if block code will have the higher priority over the others
    display_poll (); // answer requests of the display (KingMeter / Bafang wait for an answer)

    ui16_TIM2_counter = TIM2_GetCounter ();
    if ((ui16_TIM2_counter - ui16_motor_fast_controller_counter) > 78) // every 10ms
    {
      ui16_motor_fast_controller_counter = ui16_TIM2_counter;
      motor_fast_controller ();
      continue;
    }

    ui16_TIM2_counter = TIM2_GetCounter ();
    if ((ui16_TIM2_counter - ui16_motor_controller_counter) > 781) // every 100ms
    {
      ui16_motor_controller_counter = ui16_TIM2_counter;
      motor_controller ();
      continue;
    }

    ui16_TIM2_counter = TIM2_GetCounter ();
    if ((ui16_TIM2_counter - ui16_ebike_app_controller_counter) > 781) // every 150ms
    {
      ui16_ebike_app_controller_counter = ui16_TIM2_counter;
      ebike_app_controller ();
      continue;
    }
  }

  return 0;
}

