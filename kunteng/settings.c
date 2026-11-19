/*
 * Runtime settings for the KT firmware: read the settings block from the data EEPROM, fall back to the
 * compile time defaults of main.h. See settings.h for the block layout.
 *
 * This file is part of the KT firmware, GPL v3.
 */

#include <stdint.h>
#include "stm8s.h"
#include "main.h"
#include "settings.h"

struct_kt_settings kt_settings;

#ifndef PHASE_CURRENT_FACTOR_X10
#define PHASE_CURRENT_FACTOR_X10 25 // phase current may be 2.5 x the battery current limit
#endif

static uint8_t read_eeprom (uint8_t ui8_offset)
{
  return *((volatile uint8_t *) (SETTINGS_BLOCK_ADDRESS + ui8_offset));
}

static void load_defaults (void)
{
  kt_settings.ui8_cells = BATTERY_LI_ION_CELLS_NUMBER;
#if defined (MOTOR_CURRENT_MAX_A)
  kt_settings.ui8_motor_current_a = MOTOR_CURRENT_MAX_A;
#else
  kt_settings.ui8_motor_current_a = ADC_MOTOR_CURRENT_MAX / 2;
#endif
  kt_settings.ui8_regen_current_a = ADC_MOTOR_REGEN_CURRENT_MAX / 2;
  kt_settings.ui8_throttle_type = EBIKE_THROTTLE_TYPE;
  kt_settings.ui8_flags = 0;
#if defined (MOTOR_TYPE_DIRECT_DRIVE)
  kt_settings.ui8_flags |= KT_FLAG_DIRECT_DRIVE;
#endif
#if (PAS_DIRECTION == PAS_DIRECTION_LEFT)
  kt_settings.ui8_flags |= KT_FLAG_PAS_LEFT;
#endif
#if defined (BATTERY_OVER_VOLTAGE_PROTECTION)
  kt_settings.ui8_flags |= KT_FLAG_OVER_VOLTAGE_PROTECTION;
#endif
#if defined (DISABLE_PHASE_CURRENT_LIMIT)
  kt_settings.ui8_flags |= KT_FLAG_NO_PHASE_CURRENT_LIMIT;
#endif
#if defined (DISABLE_HALL_LEARNING)
  kt_settings.ui8_flags |= KT_FLAG_NO_HALL_LEARNING;
#endif
  kt_settings.ui8_pas_magnets = PAS_NUMBER_MAGNETS;
  kt_settings.ui8_pas_max_cadence = PAS_MAX_CADENCE_RPM;
  kt_settings.ui8_assist_percent[0] = (uint8_t) (PAS_ASSIST_LEVEL_1 * 100.0 + 0.5);
  kt_settings.ui8_assist_percent[1] = (uint8_t) (PAS_ASSIST_LEVEL_2 * 100.0 + 0.5);
  kt_settings.ui8_assist_percent[2] = (uint8_t) (PAS_ASSIST_LEVEL_3 * 100.0 + 0.5);
  kt_settings.ui8_assist_percent[3] = (uint8_t) (PAS_ASSIST_LEVEL_4 * 100.0 + 0.5);
  kt_settings.ui8_assist_percent[4] = (uint8_t) (PAS_ASSIST_LEVEL_5 * 100.0 + 0.5);
  kt_settings.ui8_rotor_offset_angle = MOTOR_ROTOR_OFFSET_ANGLE;
  kt_settings.ui8_foc_angle_adjust = FOC_READ_ID_CURRENT_ANGLE_ADJUST;
  kt_settings.ui8_erps_interpolation = MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES;
  kt_settings.ui8_default_max_speed = DEFAULT_VALUE_MAX_SPEED;
  kt_settings.ui8_default_wheel_size = DEFAULT_VALUE_WHEEL_SIZE;
  kt_settings.ui8_controller_type = CONTROLLER_TYPE;
  kt_settings.ui8_phase_current_factor_x10 = PHASE_CURRENT_FACTOR_X10;
}

// returns 1 if the EEPROM block is valid and in range
static uint8_t load_block (void)
{
  uint8_t ui8_i;
  uint8_t ui8_sum = 0;
  uint8_t ui8_p;

  if ((read_eeprom (0) != 'K') || (read_eeprom (1) != 'T') ||
      (read_eeprom (2) != SETTINGS_BLOCK_VERSION) || (read_eeprom (3) != SETTINGS_BLOCK_LENGTH))
  {
    return 0;
  }
  for (ui8_i = 0; ui8_i < (SETTINGS_BLOCK_LENGTH - 1); ui8_i++) { ui8_sum += read_eeprom (ui8_i); }
  if ((uint8_t) ~ui8_sum != read_eeprom (SETTINGS_BLOCK_LENGTH - 1)) { return 0; }

  // range checks: a value out of range rejects the whole block (better the known defaults than a guess)
  if ((read_eeprom (4) < 7) || (read_eeprom (4) > 16)) { return 0; }
  if ((read_eeprom (5) < 1) || (read_eeprom (5) > 30)) { return 0; }
  if (read_eeprom (6) > 15) { return 0; }
  if ((read_eeprom (7) != EBIKE_THROTTLE_TYPE_THROTTLE_PAS) && (read_eeprom (7) != EBIKE_THROTTLE_TYPE_TORQUE_SENSOR)) { return 0; }
  if ((read_eeprom (9) < 1) || (read_eeprom (9) > 64)) { return 0; }
  if ((read_eeprom (10) < 30) || (read_eeprom (10) > 150)) { return 0; }
  for (ui8_i = 11; ui8_i <= 15; ui8_i++) { if (read_eeprom (ui8_i) > 100) { return 0; } }
  if ((read_eeprom (18) < 5) || (read_eeprom (18) > 100)) { return 0; }
  if ((read_eeprom (22) < 10) || (read_eeprom (22) > 40)) { return 0; }

  kt_settings.ui8_cells = read_eeprom (4);
  kt_settings.ui8_motor_current_a = read_eeprom (5);
  kt_settings.ui8_regen_current_a = read_eeprom (6);
  kt_settings.ui8_throttle_type = read_eeprom (7);
  kt_settings.ui8_flags = read_eeprom (8);
  kt_settings.ui8_pas_magnets = read_eeprom (9);
  kt_settings.ui8_pas_max_cadence = read_eeprom (10);
  for (ui8_i = 0; ui8_i < 5; ui8_i++) { kt_settings.ui8_assist_percent[ui8_i] = read_eeprom (11 + ui8_i); }
  kt_settings.ui8_rotor_offset_angle = read_eeprom (16);
  kt_settings.ui8_foc_angle_adjust = read_eeprom (17);
  kt_settings.ui8_erps_interpolation = read_eeprom (18);
  ui8_p = read_eeprom (19);
  if ((ui8_p >= 5) && (ui8_p <= 70)) { kt_settings.ui8_default_max_speed = ui8_p; }
  ui8_p = read_eeprom (20);
  if (ui8_p <= 0x1e) { kt_settings.ui8_default_wheel_size = ui8_p; }
  kt_settings.ui8_controller_type = read_eeprom (21);
  kt_settings.ui8_phase_current_factor_x10 = read_eeprom (22);
  return 1;
}

static void calc_derived (void)
{
  uint16_t ui16_temp;
  uint8_t ui8_cells = kt_settings.ui8_cells;

  // battery: 4.2 V and 3.0 V per cell, 0.272 V per ADC step
  kt_settings.ui8_adc_battery_voltage_max = (uint8_t) ((((uint16_t) ui8_cells) * 420) / 27);
  kt_settings.ui8_adc_battery_voltage_min = (uint8_t) ((((uint16_t) ui8_cells) * 300) / 27);
  if (ui8_cells <= 8) { kt_settings.ui8_communications_battery_voltage = 24; }
  else if (ui8_cells <= 11) { kt_settings.ui8_communications_battery_voltage = 36; }
  else { kt_settings.ui8_communications_battery_voltage = 48; }
  // display bars: volts * 256 (same scale as ADC * ADC_BATTERY_VOLTAGE_K)
  kt_settings.ui16_battery_pack_volts_x256[0] = (uint16_t) (LI_ION_CELL_VOLTS_100 * 256.0) * ui8_cells;
  kt_settings.ui16_battery_pack_volts_x256[1] = (uint16_t) (LI_ION_CELL_VOLTS_80 * 256.0) * ui8_cells;
  kt_settings.ui16_battery_pack_volts_x256[2] = (uint16_t) (LI_ION_CELL_VOLTS_40 * 256.0) * ui8_cells;
  kt_settings.ui16_battery_pack_volts_x256[3] = (uint16_t) (LI_ION_CELL_VOLTS_20 * 256.0) * ui8_cells;

  // PAS: (1 / (rpm / 60)) / (magnets * 64 us)
  kt_settings.ui16_pas_max_cadence_ticks = 6250 / kt_settings.ui8_pas_magnets;      // 150 rpm
  kt_settings.ui16_pas_min_cadence_ticks = (uint16_t) (156250L / kt_settings.ui8_pas_magnets); // 6 rpm
  if (kt_settings.ui8_pas_magnets < 3) { kt_settings.ui16_pas_min_cadence_ticks = 63000; } // keep below the 16 bit counter limit

  // phase current limit in ADC steps of 0.5 A (the 8 bit total current reading)
  ui16_temp = (((uint16_t) kt_settings.ui8_motor_current_a) * 2 * kt_settings.ui8_phase_current_factor_x10) / 10;
  kt_settings.ui8_phase_current_max = (ui16_temp > 250) ? 250 : (uint8_t) ui16_temp;
}

void settings_init (void)
{
  load_defaults ();
  kt_settings.ui8_from_eeprom = load_block ();
  calc_derived ();
}

uint8_t settings_flag (uint8_t ui8_flag)
{
  return (kt_settings.ui8_flags & ui8_flag) ? 1 : 0;
}
