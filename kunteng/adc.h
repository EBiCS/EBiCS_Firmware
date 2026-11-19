/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _ADC_H
#define _ADC_H

#include "main.h"

#define ADC1_CHANNEL_PHASE_CURRENT_B 			ADC1_CHANNEL_5
#define ADC1_CHANNEL_MOTOR_TOTAL_CURRENT		ADC1_CHANNEL_6
#define ADC1_CHANNEL_MOTOR_TOTAL_CURRENT_FILTERED	ADC1_CHANNEL_8
#define ADC1_CHANNEL_BATTERY_VOLTAGE			ADC1_CHANNEL_9
#define ADC1_CHANNEL_THROTTLE				ADC1_CHANNEL_4

#define UI8_ADC_BATTERY_VOLTAGE 			(*(uint8_t*)(0x53F2))
#define UI8_ADC_MOTOR_TOTAL_CURRENT			(*(uint8_t*)(0x53F0))
#define UI8_ADC_PHASE_B_CURRENT 			(*(uint8_t*)(0x53EA))

extern uint8_t adc_throttle_busy_flag;
extern uint8_t ui8_BatteryVoltage;
extern uint8_t ui8_BatteryCurrent;
extern uint8_t ui8_adc_throttle_value;
extern uint8_t ui8_motor_total_current_offset;
extern uint16_t ui16_motor_total_current_offset_10b;

void adc_init (void);
inline void adc_trigger (void);
uint8_t ui8_adc_read_phase_B_current (void);
uint16_t ui16_adc_read_phase_B_current (void);
uint8_t ui8_adc_read_throttle (void);
uint8_t ui8_adc_read_motor_total_current (void);
uint16_t ui16_adc_read_motor_total_current_10b (void);
uint8_t ui8_adc_read_battery_voltage (void);

#endif /* _ADC_H */
