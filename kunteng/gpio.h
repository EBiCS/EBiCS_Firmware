/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

/* Connections:
 *
 * Motor PHASE_A: yellow wire
 * Motor PHASE_B: green wire
 * Motor PHASE_C: blue wire
 *
 *
 * PIN		      | IN/OUT|Function
 * ----------------------------------------------------------
 * PB5  (ADC_AIN5)    | in  | phase_B_current
 * PB6  (ADC_AIN6)    | in  | motor_total_current
 * PE7  (ADC_AIN8)    | in  | motor_total_current_filtered
 * PD7                | in  | motor_total_over_current
 *
 * PE6  (ADC_AIN9)    | in  | battery_voltage
 *
 * PE0                | in  | Hall_sensor_A
 * PE1                | in  | Hall_sensor_B
 * PE2                | in  | Hall_sensor_C
 *
 * PB2  (TIM1_CH3N)   | out | PWM_phase_A_low
 * PB1  (TIM1_CH2N)   | out | PWM_phase_B_low
 * PB0  (TIM1_CH1N)   | out | PWM_phase_C_low
 * PC3  (TIM1_CH3)    | out | PWM_phase_A_high
 * PC2  (TIM1_CH2)    | out | PWM_phase_B_high
 * PC1  (TIM1_CH1)    | out | PWM_phase_C_high
 *
 * PD5  (UART2_TX)    | out | usart_tx
 * PD6  (UART2_RX)    | out | usart_rx
 *
 * PA4                | in  | brake
 * PB4  (ADC_AIN4)    | in  | throttle
 * PD0                | in  | PAS
 * PC5                | in  | wheel speed
 *
 *
 */

#ifndef _GPIO_H_
#define _GPIO_H_

#include "main.h"
#include "stm8s_gpio.h"

#define CURRENT_PHASE_B__PIN      GPIO_PIN_5
#define CURRENT_PHASE_B__PORT     GPIOB
#define CURRENT_MOTOR_TOTAL__PIN  GPIO_PIN_6
#define CURRENT_MOTOR_TOTAL__PORT GPIOB
#define CURRENT_MOTOR_TOTAL_FILTERED__PIN GPIO_PIN_7
#define CURRENT_MOTOR_TOTAL_FILTRED__PORT GPIOE
#define CURRENT_MOTOR_TOTAL_OVER__PIN  GPIO_PIN_7
#define CURRENT_MOTOR_TOTAL_OVER__PORT GPIOD

#define BATTERY_VOLTAGE__PIN      GPIO_PIN_6
#define BATTERY_VOLTAGE__PORT     GPIOE

#define HALL_SENSOR_A__PIN      GPIO_PIN_0
#define HALL_SENSOR_B__PIN      GPIO_PIN_1
#define HALL_SENSOR_C__PIN      GPIO_PIN_2

#define HALL_SENSOR_A__PORT       GPIOE
#define HALL_SENSOR_B__PORT       GPIOE
#define HALL_SENSOR_C__PORT       GPIOE
#define HALL_SENSORS__PORT        GPIOE
#define HALL_SENSORS_MASK 	      (HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN)

#define PMW_PHASE_A_LOW__PIN      GPIO_PIN_2
#define PMW_PHASE_A_LOW__PORT     GPIOB
#define PMW_PHASE_B_LOW__PIN      GPIO_PIN_1
#define PMW_PHASE_B_LOW__PORT     GPIOB
#define PMW_PHASE_C_LOW__PIN      GPIO_PIN_0
#define PMW_PHASE_C_LOW__PORT     GPIOB
#define PMW_PHASE_A_HIGH__PIN     GPIO_PIN_3
#define PMW_PHASE_A_HIGH__PORT    GPIOC
#define PMW_PHASE_B_HIGH__PIN     GPIO_PIN_2
#define PMW_PHASE_B_HIGH__PORT    GPIOC
#define PMW_PHASE_C_HIGH__PIN     GPIO_PIN_1
#define PMW_PHASE_C_HIGH__PORT    GPIOC

#define UART2_TX__PIN             GPIO_PIN_5
#define UART2_TX__PORT            GPIOD
#define UART2_RX__PIN             GPIO_PIN_6
#define UART2_RX__PORT            GPIOD

#define BRAKE__PIN                GPIO_PIN_4
#define BRAKE__PORT               GPIOA

#define THROTTLE__PIN             GPIO_PIN_4
#define THROTTLE__PORT            GPIOB

#define PAS__PIN                  GPIO_PIN_0
#define PAS__PORT                 GPIOD

#define WHEEL_SPEED_SENSOR__PIN   GPIO_PIN_5
#define WHEEL_SPEED_SENSOR__PORT  GPIOC

#define DEBUG__PIN                GPIO_PIN_2
#define DEBUG__PORT               GPIOD

void gpio_init (void);
void debug_pin_init (void);
void debug_pin_set (void);
void debug_pin_reset (void);
void PAS_init (void);
void SPEED_init (void);

#endif /* GPIO_H_ */
