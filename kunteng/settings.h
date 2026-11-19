/*
 * Runtime settings for the KT firmware.
 *
 * The EBiCS Setup app (PC and Android) writes a small settings block into the data EEPROM over SWIM.
 * At power up the firmware reads it; without a valid block the compile time defaults from main.h are used.
 * So one prebuilt firmware works for every bike, and settings can be changed without reinstalling.
 *
 * Block layout (32 bytes at EEPROM address SETTINGS_BLOCK_ADDRESS), must match setup_app/kunteng_core.py
 * and android_app/lib/kunteng/kt_settings.dart:
 *   0  'K'                       16 hall sensor angle offset (/256)
 *   1  'T'                       17 FOC current sample angle (/256)
 *   2  version (1)               18 erps for sine wave mode
 *   3  length (32)               19 speed limit default (km/h)
 *   4  battery cells             20 wheel size default (KT display code)
 *   5  motor current (A)         21 controller type (1 = S06S, 2 = S12S)
 *   6  regen current (A)         22 phase current factor (x10, phase limit = factor * motor current)
 *   7  throttle type (1/2)       23 ... 30 reserved (0)
 *   8  flags (KT_FLAG_*)         31 checksum: ~(sum of bytes 0 ... 30)
 *   9  PAS magnets
 *   10 cadence for full PAS power (rpm)
 *   11 ... 15 assist level 1 ... 5 (percent)
 *
 * This file is part of the KT firmware, GPL v3.
 */

#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include <stdint.h>

#define SETTINGS_BLOCK_ADDRESS 0x4080 // second 128 byte block of the data EEPROM (the first one holds the display values)
#define SETTINGS_BLOCK_LENGTH 32
#define SETTINGS_BLOCK_VERSION 1

#define KT_FLAG_DIRECT_DRIVE            0x01
#define KT_FLAG_PAS_LEFT                0x02
#define KT_FLAG_OVER_VOLTAGE_PROTECTION 0x04
#define KT_FLAG_NO_PHASE_CURRENT_LIMIT  0x08
#define KT_FLAG_NO_HALL_LEARNING        0x10

typedef struct
{
  uint8_t ui8_cells;
  uint8_t ui8_motor_current_a;
  uint8_t ui8_regen_current_a;
  uint8_t ui8_throttle_type;
  uint8_t ui8_flags;
  uint8_t ui8_pas_magnets;
  uint8_t ui8_pas_max_cadence;
  uint8_t ui8_assist_percent[5];
  uint8_t ui8_rotor_offset_angle;
  uint8_t ui8_foc_angle_adjust;
  uint8_t ui8_erps_interpolation;
  uint8_t ui8_default_max_speed;
  uint8_t ui8_default_wheel_size;
  uint8_t ui8_controller_type;
  uint8_t ui8_phase_current_factor_x10;

  // derived at start up
  uint8_t ui8_adc_battery_voltage_max;
  uint8_t ui8_adc_battery_voltage_min;
  uint8_t ui8_communications_battery_voltage;
  uint16_t ui16_battery_pack_volts_x256[4]; // 100, 80, 40, 20 % for the display bars
  uint16_t ui16_pas_max_cadence_ticks; // shortest PAS period accepted (150 rpm)
  uint16_t ui16_pas_min_cadence_ticks; // longer PAS periods mean "not pedalling" (6 rpm)
  uint8_t ui8_phase_current_max; // ADC steps of 0.5 A
  uint8_t ui8_from_eeprom; // 1 when the block was valid
} struct_kt_settings;

extern struct_kt_settings kt_settings;

void settings_init (void); // call before motor_init / eeprom_init
uint8_t settings_flag (uint8_t ui8_flag);

#endif /* _SETTINGS_H_ */
