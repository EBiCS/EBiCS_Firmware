/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include "motor.h"

#include <stdint.h>
#include <stdio.h>
#include "interrupts.h"
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "motor.h"
#include "ebike_app.h"
#include "gpio.h"
#include "pwm.h"
#include "config.h"
#include "adc.h"
#include "utils.h"
#include "uart.h"
#include "adc.h"
#include "watchdog.h"
#include "stm8s_exti.h"

#define SVM_TABLE_LEN 256

uint8_t ui8_svm_table [SVM_TABLE_LEN] =
{
    127	,
    133	,
    138	,
    144	,
    149	,
    154	,
    160	,
    165	,
    170	,
    176	,
    181	,
    186	,
    191	,
    197	,
    202	,
    207	,
    212	,
    217	,
    222	,
    227	,
    231	,
    236	,
    239	,
    240	,
    242	,
    243	,
    244	,
    245	,
    247	,
    248	,
    249	,
    250	,
    250	,
    251	,
    252	,
    253	,
    253	,
    254	,
    254	,
    254	,
    255	,
    255	,
    255	,
    255	,
    255	,
    255	,
    254	,
    254	,
    254	,
    253	,
    253	,
    252	,
    251	,
    251	,
    250	,
    249	,
    248	,
    247	,
    246	,
    245	,
    243	,
    242	,
    241	,
    239	,
    238	,
    239	,
    241	,
    242	,
    243	,
    245	,
    246	,
    247	,
    248	,
    249	,
    250	,
    251	,
    251	,
    252	,
    253	,
    253	,
    254	,
    254	,
    254	,
    255	,
    255	,
    255	,
    255	,
    255	,
    255	,
    254	,
    254	,
    254	,
    253	,
    253	,
    252	,
    251	,
    250	,
    250	,
    249	,
    248	,
    247	,
    245	,
    244	,
    243	,
    242	,
    240	,
    239	,
    236	,
    231	,
    227	,
    222	,
    217	,
    212	,
    207	,
    202	,
    197	,
    191	,
    186	,
    181	,
    176	,
    170	,
    165	,
    160	,
    154	,
    149	,
    144	,
    138	,
    133	,
    127	,
    122	,
    116	,
    111	,
    106	,
    100	,
    95	,
    89	,
    84	,
    79	,
    74	,
    68	,
    63	,
    58	,
    53	,
    48	,
    43	,
    38	,
    33	,
    28	,
    23	,
    18	,
    16	,
    14	,
    13	,
    12	,
    10	,
    9	,
    8	,
    7	,
    6	,
    5	,
    4	,
    3	,
    3	,
    2	,
    1	,
    1	,
    1	,
    0	,
    0	,
    0	,
    0	,
    0	,
    0	,
    0	,
    0	,
    0	,
    1	,
    1	,
    2	,
    2	,
    3	,
    4	,
    5	,
    6	,
    6	,
    8	,
    9	,
    10	,
    11	,
    12	,
    14	,
    15	,
    17	,
    15	,
    14	,
    12	,
    11	,
    10	,
    9	,
    8	,
    6	,
    6	,
    5	,
    4	,
    3	,
    2	,
    2	,
    1	,
    1	,
    0	,
    0	,
    0	,
    0	,
    0	,
    0	,
    0	,
    0	,
    0	,
    1	,
    1	,
    1	,
    2	,
    3	,
    3	,
    4	,
    5	,
    6	,
    7	,
    8	,
    9	,
    10	,
    12	,
    13	,
    14	,
    16	,
    18	,
    23	,
    28	,
    33	,
    38	,
    43	,
    48	,
    53	,
    58	,
    63	,
    68	,
    74	,
    79	,
    84	,
    89	,
    95	,
    100	,
    106	,
    111	,
    116	,
    122
};

#include "settings.h"

/*
 * Changes against casainho's original (see kunteng/CHANGES.md):
 * - settings at runtime (settings.c) instead of compile time constants
 * - 9 bit PWM: duty cycle 0 ... 508 and 9 bit compare values (was 8 bit shifted left)
 * - interpolation: angle advanced by a fixed step every PWM cycle, step from the sum of the last 6 hall
 *   sectors (updated at every hall edge, was once per electrical revolution and a division every cycle),
 *   never runs more than one sector past the last hall edge
 * - hall position learning: real angle of every hall edge measured at steady speed
 * - FOC angle correction from both zero crossings of the phase B current (was one): twice as fast and
 *   an ADC offset no longer shifts the angle
 * - phase current limit: estimated phase current = battery current / duty cycle, protects the MOSFETs
 *   when starting under load
 * - current controller as PI every 10 ms (was an integrator every 100 ms)
 */

#define PWM_DUTY_MAX 508            // 9 bit (timer period 512, center aligned)
#define PWM_DUTY_MIDDLE 254
#define FOC_ID_ZERO_HIGH 127        // phase B current ADC dead band around zero current
#define FOC_ID_ZERO_LOW 125

// hall state -> nominal angle of its rising edge (256 = 360 deg) and the next state when turning forward
static const uint8_t ui8_hall_nominal_angle[8] = {0, 170, 85, 127, 0, 212, 42, 0};
static const uint8_t ui8_hall_next_state[8] = {0, 5, 3, 1, 6, 4, 2, 0};

uint16_t ui16_PWM_cycles_counter = 0;            // PWM cycles since the hall state 1 edge
uint16_t ui16_PWM_cycles_counter_6 = 0;          // PWM cycles since the last hall edge
uint16_t ui16_PWM_cycles_counter_total = 0xffff; // PWM cycles of the last electrical revolution
uint16_t ui16_motor_speed_erps = 0;
uint8_t ui8_sinewave_table_index = 0;
uint8_t ui8_motor_rotor_absolute_angle;           // angle of the last hall edge
uint8_t ui8_motor_rotor_angle;
volatile uint8_t ui8_angle_correction = 127;
uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;
volatile uint8_t ui8_motor_state = MOTOR_STATE_STOP;
uint8_t ui8_motor_startup_counter;
uint8_t ui8_motor_controller_state = MOTOR_CONTROLLER_STATE_OK;
uint8_t ui8_hall_sensors = 0;
uint8_t ui8_hall_sensors_last = 0;
uint8_t ui8_half_erps_flag = 0;
uint8_t ui8_flag_foc_read_id_current = 0;
uint8_t ui8_flag_foc_read_id_current_2 = 0;
uint8_t ui8_adc_id_current = 0;

// interpolation
uint16_t ui16_rotor_angle_x256;                   // 8.8 fixed point
uint16_t ui16_angle_step_x256 = 0;                // per PWM cycle
uint8_t ui8_sector_angle_limit = 42;              // max angle past the last hall edge
uint16_t ui16_sector_cycles[8];                   // PWM cycles spent in each hall state (last time)
uint16_t ui16_period_sum = 0;                     // sum of the last 6 sectors = one electrical revolution
uint8_t ui8_valid_edges = 0;                      // consecutive forward hall edges since the last reset

// hall learning: snapshot of one revolution for the main loop, learned edge offsets (/256)
volatile uint8_t ui8_learn_snapshot_ready = 0;
uint16_t ui16_learn_snapshot[8];
uint16_t ui16_learn_snapshot_sum;
volatile int8_t i8_hall_offset[8];
int16_t i16_hall_offset_x16[8];
uint16_t ui16_last_total_for_learning = 0;

// current / voltage measurement
uint8_t ui8_adc_target_motor_current_max;
uint8_t ui8_adc_target_motor_regen_current_max;
uint8_t ui8_adc_motor_total_current;
uint8_t ui8_motor_total_current_offset;
uint16_t ui16_motor_total_current_offset_10b;
int16_t i16_motor_current_filtered_10b; // 0.125 A per step (was int8_t: wrapped above 15.9 A)
uint16_t ui16_adc_motor_current_accumulated_10b = ADC_MOTOR_CURRENT_MAX_MED_10B;
uint16_t ui16_adc_motor_current_filtered_10b;
uint16_t ui16_adc_battery_voltage_accumulated = 0;
uint8_t ui8_adc_battery_voltage_filtered;
uint8_t ui8_motor_controller_error = MOTOR_CONTROLLER_ERROR_EMPTY;

// duty cycle (9 bit)
volatile uint16_t ui16_duty_cycle = 0;
uint16_t ui16_duty_cycle_target;
uint16_t ui16_duty_cycle_ramp_up_inverse_step;
uint16_t ui16_duty_cycle_ramp_down_inverse_step;
uint16_t ui16_counter_duty_cycle_ramp_up = 0;
uint16_t ui16_counter_duty_cycle_ramp_down = 0;
uint16_t ui16_pwm_duty_cycle_duty_cycle_controller;
uint16_t ui16_value_a;
uint16_t ui16_value_b;
uint16_t ui16_value_c;
uint8_t ui8_phase_current_limit_on = 1;

// targets from the e-bike application
uint16_t ui16_target_erps = 0;
volatile uint16_t ui16_target_erps_max = MOTOR_OVER_SPEED_ERPS;
uint16_t ui16_target_current_10b = 0;
uint16_t ui16_duty_cycle_speed_controller = PWM_DUTY_MAX;
int16_t i16_current_error_last = 0;
int16_t i16_current_pi_fraction = 0;

uint8_t ui8_first_time_run_flag = 1;

// PAS and wheel speed sensor timing
uint8_t ui8_pas_state;
uint8_t ui8_pas_state_old;
uint16_t ui16_pas_counter;
uint16_t ui16_pas_on_time_counter;
uint16_t ui16_pas_off_time_counter;
uint8_t ui8_wheel_speed_sensor_state;
uint8_t ui8_wheel_speed_sensor_state_old;
uint16_t ui16_wheel_speed_sensor_counter;

uint8_t ui8_over_current_counter = 0;
uint8_t ui8_over_current_retries = 0;

// functions prototypes
void do_battery_voltage_protection (void);
uint16_t motor_current_controller (void);
uint16_t motor_speed_controller (void);
void do_motor_state_machine (void);
void calc_motor_current_filtered (void);
void do_over_current_recovery (void);
void do_hall_learning (void);

static uint16_t pwm_value (uint8_t ui8_table, uint16_t ui16_duty)
{
  // table value 0 ... 255 around 127, scaled by the duty cycle to 0 ... 508 around 254
  if (ui8_table > 127) { return PWM_DUTY_MIDDLE + ((((uint16_t) (ui8_table - 127)) * ui16_duty) >> 8); }
  return PWM_DUTY_MIDDLE - ((((uint16_t) (127 - ui8_table)) * ui16_duty) >> 8);
}

static void reset_interpolation (void)
{
  uint8_t ui8_i;
  for (ui8_i = 0; ui8_i < 8; ui8_i++) { ui16_sector_cycles[ui8_i] = PWM_CYCLES_COUNTER_MAX; }
  ui16_period_sum = 6 * PWM_CYCLES_COUNTER_MAX;
  ui8_valid_edges = 0;
  ui16_angle_step_x256 = 0;
}

// runs every 64us (PWM frequency)
void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM1_UPD_OVF_TRG_BRK_IRQHANDLER)
{
  uint8_t ui8_temp;
  uint8_t ui8_previous_state;
  uint8_t ui8_next;
  int16_t i16_temp;

  /****************************************************************************/
  // trigger ADC conversion of all channels (scan conversion, buffered)
  ADC1->CSR &= 0x09; // clear EOC flag first (selected also channel 9)
  ADC1->CR1 |= ADC1_CR1_ADON; // Start ADC1 conversion

  /****************************************************************************/
  // hall sensors: rotor angle at every edge, sector timing, speed
  ui8_hall_sensors = ((uint8_t) HALL_SENSORS__PORT->IDR) & (HALL_SENSORS_MASK);
  if (ui8_hall_sensors != ui8_hall_sensors_last)
  {
    if ((ui8_hall_sensors == 0) || (ui8_hall_sensors == 7)) { goto hall_done; } // invalid state: ignore

    ui8_previous_state = ui8_hall_sensors_last;
    ui8_hall_sensors_last = ui8_hall_sensors;

    // sector timing: only for forward steps, everything else restarts the measurement
    if ((ui8_previous_state >= 1) && (ui8_previous_state <= 6) &&
        (ui8_hall_next_state[ui8_previous_state] == ui8_hall_sensors) &&
        (ui16_PWM_cycles_counter_6 < PWM_CYCLES_COUNTER_MAX))
    {
      ui16_period_sum = ui16_period_sum - ui16_sector_cycles[ui8_previous_state] + ui16_PWM_cycles_counter_6;
      ui16_sector_cycles[ui8_previous_state] = ui16_PWM_cycles_counter_6;
      if (ui8_valid_edges < 255) { ui8_valid_edges++; }
      if (ui8_valid_edges >= 6) { ui16_angle_step_x256 = 0xffff / ui16_period_sum; }
    }
    else
    {
      reset_interpolation ();
    }

    if (ui8_hall_sensors == 1)
    {
      if (ui8_half_erps_flag == 1)
      {
        ui8_half_erps_flag = 0;
        ui16_PWM_cycles_counter_total = ui16_PWM_cycles_counter;
        ui16_PWM_cycles_counter = 0;
        ui16_motor_speed_erps = ((uint16_t) PWM_CYCLES_SECOND) / ui16_PWM_cycles_counter_total;

        // hand one steady revolution to the hall learning in the main loop
        if ((ui8_valid_edges >= 12) && (!ui8_learn_snapshot_ready) &&
            (ui16_PWM_cycles_counter_total + (ui16_PWM_cycles_counter_total >> 4) > ui16_last_total_for_learning) &&
            (ui16_PWM_cycles_counter_total < ui16_last_total_for_learning + (ui16_last_total_for_learning >> 4)))
        {
          for (ui8_temp = 1; ui8_temp <= 6; ui8_temp++) { ui16_learn_snapshot[ui8_temp] = ui16_sector_cycles[ui8_temp]; }
          ui16_learn_snapshot_sum = ui16_period_sum;
          ui8_learn_snapshot_ready = 1;
        }
        ui16_last_total_for_learning = ui16_PWM_cycles_counter_total;
      }

      // block commutation at low speed, sine wave with interpolation above
      if (ui16_motor_speed_erps > kt_settings.ui8_erps_interpolation)
      {
        if (ui8_motor_commutation_type == BLOCK_COMMUTATION)
        {
          ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES;
          ui8_motor_state = MOTOR_STATE_RUNNING;
        }
      }
      else
      {
        if (ui8_motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES)
        {
          ui8_motor_commutation_type = BLOCK_COMMUTATION;
          ui8_angle_correction = 127;
        }
      }
      ui8_flag_foc_read_id_current_2 = 1; // second FOC sample, 180 deg after the first
    }
    else if (ui8_hall_sensors == 6)
    {
      ui8_half_erps_flag = 1;
      ui8_flag_foc_read_id_current = 1;
    }

    // angle of this edge: nominal + learned offset + motor specific offset
    ui8_motor_rotor_absolute_angle = (uint8_t) (ui8_hall_nominal_angle[ui8_hall_sensors] +
                                                (uint8_t) i8_hall_offset[ui8_hall_sensors] +
                                                kt_settings.ui8_rotor_offset_angle);
    ui16_rotor_angle_x256 = ((uint16_t) ui8_motor_rotor_absolute_angle) << 8;
    // the interpolation may run to the next edge plus a small margin, not further
    ui8_next = ui8_hall_next_state[ui8_hall_sensors];
    i16_temp = (int16_t) (uint8_t) (ui8_hall_nominal_angle[ui8_next] - ui8_hall_nominal_angle[ui8_hall_sensors]) + 4 +
               (int16_t) i8_hall_offset[ui8_next] - (int16_t) i8_hall_offset[ui8_hall_sensors];
    ui8_sector_angle_limit = (uint8_t) i16_temp;

    ui16_PWM_cycles_counter_6 = 0;
  }
hall_done:

  /****************************************************************************/
  // count PWM cycles, reset when the motor is stopped or near zero speed
  if (ui16_PWM_cycles_counter < ((uint16_t) PWM_CYCLES_COUNTER_MAX))
  {
    ui16_PWM_cycles_counter++;
    ui16_PWM_cycles_counter_6++;
  }
  else
  {
    ui16_PWM_cycles_counter = 0;
    ui16_PWM_cycles_counter_6 = 0;
    ui8_half_erps_flag = 0;
    ui16_motor_speed_erps = 0;
    ui16_PWM_cycles_counter_total = 0xffff;
    ui8_angle_correction = 127;
    ui8_motor_commutation_type = BLOCK_COMMUTATION;
    ui8_hall_sensors_last = 0; // this way we force execution of hall sensors code next time
    reset_interpolation ();
    ebike_app_cruise_control_stop ();
    if (ui8_motor_state == MOTOR_STATE_RUNNING) { ui8_motor_state = MOTOR_STATE_STOP; }
  }

  /****************************************************************************/
  // rotor angle and sine table index
  if ((ui8_motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES) && ui16_angle_step_x256)
  {
    ui16_rotor_angle_x256 += ui16_angle_step_x256;
    ui8_motor_rotor_angle = (uint8_t) (ui16_rotor_angle_x256 >> 8);
    if ((uint8_t) (ui8_motor_rotor_angle - ui8_motor_rotor_absolute_angle) > ui8_sector_angle_limit)
    {
      // motor is slowing down: wait at the limit for the next hall edge
      ui8_motor_rotor_angle = ui8_motor_rotor_absolute_angle + ui8_sector_angle_limit;
      ui16_rotor_angle_x256 = ((uint16_t) ui8_motor_rotor_angle) << 8;
    }
  }
  else
  {
    ui8_motor_rotor_angle = ui8_motor_rotor_absolute_angle;
  }
  ui8_sinewave_table_index = ui8_motor_rotor_angle + ui8_angle_correction;

  /****************************************************************************/
  // FOC: the phase B current must cross zero at a known angle. Read it there, at both zero crossings,
  // and move the angle correction until it does (Id = 0)
  ui8_temp = ui8_motor_rotor_angle - kt_settings.ui8_rotor_offset_angle; // angle relative to the halls
  if (ui16_motor_speed_erps > kt_settings.ui8_erps_interpolation)
  {
    if (ui8_flag_foc_read_id_current && (ui8_temp >= kt_settings.ui8_foc_angle_adjust))
    {
      ui8_flag_foc_read_id_current = 0;
      ui8_adc_id_current = UI8_ADC_PHASE_B_CURRENT;
      if (ui8_adc_id_current > FOC_ID_ZERO_HIGH) { ui8_angle_correction++; }
      else if (ui8_adc_id_current < FOC_ID_ZERO_LOW) { ui8_angle_correction--; }
    }
    if (ui8_flag_foc_read_id_current_2 && ((uint8_t) (ui8_temp - 128) >= kt_settings.ui8_foc_angle_adjust))
    {
      ui8_flag_foc_read_id_current_2 = 0;
      ui8_adc_id_current = UI8_ADC_PHASE_B_CURRENT;
      // half a revolution later the current has the opposite sign
      if (ui8_adc_id_current > FOC_ID_ZERO_HIGH) { ui8_angle_correction--; }
      else if (ui8_adc_id_current < FOC_ID_ZERO_LOW) { ui8_angle_correction++; }
    }
  }
  else
  {
    ui8_flag_foc_read_id_current = 0;
    ui8_flag_foc_read_id_current_2 = 0;
  }

  /****************************************************************************/
  // duty cycle controller: current limits, regen limits, ramps
  ui8_adc_motor_total_current = UI8_ADC_MOTOR_TOTAL_CURRENT;
  if (ui8_adc_motor_total_current > ui8_adc_target_motor_current_max) // battery current limit
  {
    if (ui16_duty_cycle > 0) { ui16_duty_cycle--; }
  }
  // phase current = battery current / duty cycle: at low duty cycle (start up, climbing slowly) the
  // phase current can be several times the battery current
  else if (ui8_phase_current_limit_on && (ui8_adc_motor_total_current > ui8_motor_total_current_offset) &&
           ((((uint16_t) (ui8_adc_motor_total_current - ui8_motor_total_current_offset)) << 8) >
            ((uint16_t) kt_settings.ui8_phase_current_max) * ((uint8_t) (ui16_duty_cycle >> 1))))
  {
    if (ui16_duty_cycle > 0) { ui16_duty_cycle--; }
  }
  // regen while the battery is full: reduce regen current
  else if ((ui8_adc_motor_total_current < ui8_motor_total_current_offset) &&
           (UI8_ADC_BATTERY_VOLTAGE >= kt_settings.ui8_adc_battery_voltage_max))
  {
    if (ui16_duty_cycle < PWM_DUTY_MAX) { ui16_duty_cycle++; }
  }
  else if (ui8_adc_motor_total_current < ui8_adc_target_motor_regen_current_max) // regen current limit
  {
    if (ui16_duty_cycle < PWM_DUTY_MAX) { ui16_duty_cycle++; }
  }
  else // no limit active: ramp to the target
  {
    if (ui16_duty_cycle_target > ui16_duty_cycle)
    {
      if (ui16_counter_duty_cycle_ramp_up++ >= ui16_duty_cycle_ramp_up_inverse_step)
      {
        ui16_counter_duty_cycle_ramp_up = 0;
        ui16_duty_cycle++;
      }
    }
    else if (ui16_duty_cycle_target < ui16_duty_cycle)
    {
      if (ui16_counter_duty_cycle_ramp_down++ >= ui16_duty_cycle_ramp_down_inverse_step)
      {
        ui16_counter_duty_cycle_ramp_down = 0;
        ui16_duty_cycle--;
      }
    }
  }

  /****************************************************************************/
  // PWM compare values, 9 bit
  ui16_value_a = pwm_value (ui8_svm_table [ui8_sinewave_table_index], ui16_duty_cycle);
  ui16_value_b = pwm_value (ui8_svm_table [(uint8_t) (ui8_sinewave_table_index + 85 /* 120 deg */)], ui16_duty_cycle);
  ui16_value_c = pwm_value (ui8_svm_table [(uint8_t) (ui8_sinewave_table_index + 171 /* 240 deg */)], ui16_duty_cycle);

  // phase A
  TIM1->CCR1H = (uint8_t) (ui16_value_a >> 8);
  TIM1->CCR1L = (uint8_t) ui16_value_a;
  // phase B
  TIM1->CCR2H = (uint8_t) (ui16_value_c >> 8);
  TIM1->CCR2L = (uint8_t) ui16_value_c;
  // phase C
  TIM1->CCR3H = (uint8_t) (ui16_value_b >> 8);
  TIM1->CCR3L = (uint8_t) ui16_value_b;

  // enable PWM signals only when MOTOR_CONTROLLER_STATE_OK
  if (ui8_motor_controller_state == MOTOR_CONTROLLER_STATE_OK)
  {
    TIM1->BKR |= TIM1_BKR_MOE;
  }

  /****************************************************************************/
  // PAS timing between positive pulses and on/off time of each pulse, in PWM cycles
  ui16_pas_counter++;
  if ((PAS__PORT->IDR & PAS__PIN) == 0)
  {
    ui8_pas_state = 0;
    ui16_pas_off_time_counter++;
  }
  else
  {
    ui8_pas_state = 1;
    ui16_pas_on_time_counter++;
  }

  if (ui8_pas_state != ui8_pas_state_old)
  {
    ui8_pas_state_old = ui8_pas_state;
    if (ui8_pas_state == 1) // 0 -> 1 transition
    {
      if (ui16_pas_counter < kt_settings.ui16_pas_max_cadence_ticks) { ui16_pas_counter = kt_settings.ui16_pas_max_cadence_ticks; }
      ui16_pas_pwm_cycles_ticks = ui16_pas_counter;
      ui16_pas_counter = 0;
    }
    else
    {
      ui8_temp = (ui16_pas_on_time_counter > ui16_pas_off_time_counter) ? 1 : 0;
      if (kt_settings.ui8_flags & KT_FLAG_PAS_LEFT) { ui8_temp = ui8_temp ? 0 : 1; }
      ui8_pas_direction = ui8_temp;
      ui16_pas_off_time_counter = 0;
      ui16_pas_on_time_counter = 0;
    }
  }

  // limit min PAS cadence
  if (ui16_pas_counter > kt_settings.ui16_pas_min_cadence_ticks)
  {
    ui16_pas_pwm_cycles_ticks = kt_settings.ui16_pas_min_cadence_ticks;
    ui16_pas_counter = 0;
    ui16_pas_on_time_counter = 0;
    ui16_pas_off_time_counter = 0;
    ui8_pas_direction = 1;
  }

  /****************************************************************************/
  // wheel speed sensor timing between positive pulses, in PWM cycles
  ui16_wheel_speed_sensor_counter++;
  if ((WHEEL_SPEED_SENSOR__PORT->IDR & WHEEL_SPEED_SENSOR__PIN) == 0) { ui8_wheel_speed_sensor_state = 0; }
  else { ui8_wheel_speed_sensor_state = 1; }

  if (ui8_wheel_speed_sensor_state != ui8_wheel_speed_sensor_state_old)
  {
    ui8_wheel_speed_sensor_state_old = ui8_wheel_speed_sensor_state;
    if (ui8_wheel_speed_sensor_state == 1)
    {
      ui16_wheel_speed_sensor_pwm_cycles_ticks = ui16_wheel_speed_sensor_counter;
      ui16_wheel_speed_sensor_counter = 0;
      ui8_wheel_speed_sensor_is_disconnected = 0;
    }
  }

  if (ui16_wheel_speed_sensor_counter > ((uint16_t) WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS))
  {
    ui16_wheel_speed_sensor_pwm_cycles_ticks = (uint16_t) WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS;
    ui16_wheel_speed_sensor_counter = 0;
    ui8_wheel_speed_sensor_is_disconnected = 1;
  }

  /****************************************************************************/
  // reload watchdog timer, every PWM cycle to avoid automatic reset of the microcontroller
  if (ui8_first_time_run_flag)
  { // from the init of watchdog up to first reset on PWM cycle interrupt,
    // it can take up to 250ms and so we need to init here inside the PWM cycle
    ui8_first_time_run_flag = 0;
    watchdog_init ();
  }
  else
  {
    IWDG->KR = IWDG_KEY_REFRESH; // reload watch dog timer counter
  }

  // clears the TIM1 interrupt TIM1_IT_UPDATE pending bit
  TIM1->SR1 = (uint8_t)(~(uint8_t)TIM1_IT_UPDATE);
}

/****************************************************************************/
// slow tasks

// called every 100ms
void motor_controller (void)
{
  do_over_current_recovery ();
  do_motor_state_machine ();
  do_battery_voltage_protection ();
  ui16_duty_cycle_speed_controller = motor_speed_controller ();
}

// called every 10ms: current measurement and current controller
void motor_fast_controller (void)
{
  uint16_t ui16_target;

  calc_motor_current_filtered ();
  do_hall_learning (); // one revolution per call at most: learns in a few seconds of steady riding
#if defined (EBIKE_THROTTLE_TYPE_THROTTLE_PAS_PWM_DUTY_CYCLE)
  ui16_target = ui16_pwm_duty_cycle_duty_cycle_controller;
#else
  ui16_target = motor_current_controller ();
#endif
  if (ui16_duty_cycle_speed_controller < ui16_target) { ui16_target = ui16_duty_cycle_speed_controller; }
  motor_set_pwm_duty_cycle_target (ui16_target);
}

// the hardware over current trip used to latch until power off: retry after 1 s with the throttle
// released, max 3 times per power up
void do_over_current_recovery (void)
{
  if (!motor_controller_state_is_set (MOTOR_CONTROLLER_STATE_OVER_CURRENT))
  {
    ui8_over_current_counter = 0;
    return;
  }
  if (ui8_over_current_retries >= 3) { return; }
  if (ui8_over_current_counter < 10) { ui8_over_current_counter++; return; }
  if (!ebike_app_is_throttle_released ()) { return; }

  ui8_over_current_counter = 0;
  ui8_over_current_retries++;
  ui16_duty_cycle = 0;
  ui16_duty_cycle_target = 0;
  motor_controller_reset_state (MOTOR_CONTROLLER_STATE_OVER_CURRENT);
  motor_controller_clear_error ();
}

// hall learning: at steady speed the time spent in each hall state is proportional to its angle.
// From that, the real angle of every hall edge relative to the others (the mean offset is left to
// MOTOR_ROTOR_OFFSET_ANGLE and the FOC correction). Filtered over many revolutions, limited to +-20 (28 deg).
void do_hall_learning (void)
{
  static const uint8_t ui8_order[6] = {4, 6, 2, 3, 1, 5}; // forward sequence, starting at 0 deg
  uint8_t ui8_i;
  uint8_t ui8_state;
  uint32_t ui32_position = 0;
  int16_t i16_error[6];
  int16_t i16_mean = 0;
  int16_t i16_offset;

  if (!ui8_learn_snapshot_ready) { return; }
  if (settings_flag (KT_FLAG_NO_HALL_LEARNING) || (ui16_motor_speed_erps < 30) || (ui16_learn_snapshot_sum == 0))
  {
    ui8_learn_snapshot_ready = 0;
    return;
  }

  for (ui8_i = 0; ui8_i < 6; ui8_i++)
  {
    ui8_state = ui8_order[ui8_i];
    // edge position of this state = sum of the widths of the states before it (256 = one revolution)
    i16_error[ui8_i] = (int16_t) ((ui32_position * 256 + (ui16_learn_snapshot_sum >> 1)) / ui16_learn_snapshot_sum) -
                       (int16_t) ui8_hall_nominal_angle[ui8_state];
    i16_mean += i16_error[ui8_i];
    ui32_position += ui16_learn_snapshot[ui8_state];
  }
  ui8_learn_snapshot_ready = 0;
  i16_mean /= 6;

  for (ui8_i = 0; ui8_i < 6; ui8_i++)
  {
    ui8_state = ui8_order[ui8_i];
    i16_offset = i16_error[ui8_i] - i16_mean;
    if (i16_offset > 20) { i16_offset = 20; }
    if (i16_offset < -20) { i16_offset = -20; }
    // low pass: 1/16 of the new measurement per revolution used
    i16_hall_offset_x16[ui8_state] += i16_offset - (i16_hall_offset_x16[ui8_state] >> 4);
    i8_hall_offset[ui8_state] = (int8_t) (i16_hall_offset_x16[ui8_state] >> 4);
  }
}

void motor_disable_PWM (void)
{
  TIM1_CtrlPWMOutputs(DISABLE);
}

void motor_enable_PWM (void)
{
  TIM1_CtrlPWMOutputs(ENABLE);
}

void motor_controller_set_state (uint8_t ui8_state)
{
  ui8_motor_controller_state |= ui8_state;
}

void motor_controller_reset_state (uint8_t ui8_state)
{
  ui8_motor_controller_state &= ~ui8_state;
}

uint8_t motor_controller_state_is_set (uint8_t ui8_state)
{
  return ui8_motor_controller_state & ui8_state;
}

void hall_sensor_init (void)
{
  GPIO_Init(HALL_SENSORS__PORT,
	    (GPIO_Pin_TypeDef)(HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN),
	    GPIO_MODE_IN_FL_NO_IT);
}

void motor_init (void)
{
  // motor overcurrent pin as external input pin interrupt
  GPIO_Init(CURRENT_MOTOR_TOTAL_OVER__PORT,
	    CURRENT_MOTOR_TOTAL_OVER__PIN,
	    GPIO_MODE_IN_FL_IT); // with external interrupt

  //initialize the Interrupt sensitivity
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD,
			    EXTI_SENSITIVITY_FALL_LOW);

  // start the filters at the measured zero current and the middle battery voltage (motor_init runs after adc_init)
  ui16_adc_motor_current_accumulated_10b = ui16_motor_total_current_offset_10b << 4;
  ui16_adc_battery_voltage_accumulated = ((uint16_t) ((kt_settings.ui8_adc_battery_voltage_max +
                                                       kt_settings.ui8_adc_battery_voltage_min) >> 1)) << 6;
  ui8_phase_current_limit_on = settings_flag (KT_FLAG_NO_PHASE_CURRENT_LIMIT) ? 0 : 1;
  reset_interpolation ();

  motor_set_current_max (kt_settings.ui8_motor_current_a * 2);
  motor_set_regen_current_max (4);
  // the duty cycle has twice the steps now (9 bit): half the time per step keeps the same ramp
  motor_set_pwm_duty_cycle_ramp_up_inverse_step (PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP / 2); // each step = 64us
  motor_set_pwm_duty_cycle_ramp_down_inverse_step (PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP / 2); // each step = 64us
}

void motor_set_pwm_duty_cycle_target (uint16_t ui16_value)
{
  if (ui16_value > PWM_DUTY_MAX) { ui16_value = PWM_DUTY_MAX; }
  ui16_duty_cycle_target = ui16_value;
}

void motor_set_current_max (uint8_t ui8_value)
{
  uint16_t ui16_temp = ((uint16_t) ui8_motor_total_current_offset) + ui8_value;
  ui8_adc_target_motor_current_max = (ui16_temp > 255) ? 255 : (uint8_t) ui16_temp;
}

int16_t motor_get_current_filtered_10b (void)
{
  return i16_motor_current_filtered_10b;
}

void motor_set_regen_current_max (uint8_t ui8_value)
{
  ui8_adc_target_motor_regen_current_max = (ui8_value < ui8_motor_total_current_offset) ?
                                           (ui8_motor_total_current_offset - ui8_value) : 0;
}

void motor_set_pwm_duty_cycle_ramp_up_inverse_step (uint16_t ui16_value)
{
  ui16_duty_cycle_ramp_up_inverse_step = ui16_value;
}

void motor_set_pwm_duty_cycle_ramp_down_inverse_step (uint16_t ui16_value)
{
  ui16_duty_cycle_ramp_down_inverse_step = ui16_value;
}

uint16_t ui16_motor_get_motor_speed_erps (void)
{
  return ui16_motor_speed_erps;
}

uint16_t motor_get_er_PWM_ticks (void)
{
  return ui16_PWM_cycles_counter_total;
}

void motor_controller_set_target_speed_erps (uint16_t ui16_erps)
{
  ui16_target_erps = ui16_erps;
}

void motor_controller_set_speed_erps_max (uint16_t ui16_erps)
{
  ui16_target_erps_max = ui16_erps;
}

uint16_t motor_controller_get_target_speed_erps_max (void)
{
  return ui16_target_erps_max;
}

void motor_controller_set_target_current_10b (uint16_t ui16_current)
{
  ui16_target_current_10b = ui16_current;
}

// call every 100ms: integrating speed limiter, output is an absolute duty cycle (9 bit)
uint16_t motor_speed_controller (void)
{
  int16_t i16_error;
  int16_t i16_output;

  if (ui16_target_erps < 5) { return PWM_DUTY_MAX; } // no speed target: no limit from here

  if (ui16_target_erps > MOTOR_OVER_SPEED_ERPS) { i16_error = MOTOR_OVER_SPEED_ERPS - (int16_t) ui16_motor_speed_erps; }
  else { i16_error = (int16_t) ui16_target_erps - (int16_t) ui16_motor_speed_erps; }

  i16_output = i16_error * MOTOR_SPEED_CONTROLLER_KP;
  if (i16_output > MOTOR_SPEED_CONTROLLER_OUTPUT_MAX) i16_output = MOTOR_SPEED_CONTROLLER_OUTPUT_MAX;
  else if (i16_output < (-MOTOR_SPEED_CONTROLLER_OUTPUT_MAX)) i16_output = -MOTOR_SPEED_CONTROLLER_OUTPUT_MAX;
  i16_output >>= 4; // MOTOR_SPEED_CONTROLLER_KP is x32, and the duty cycle has 2x the steps now

  i16_output = (int16_t) ui16_duty_cycle + i16_output;
  if (i16_output > PWM_DUTY_MAX) i16_output = PWM_DUTY_MAX;
  if (i16_output < 0) i16_output = 0;
  return (uint16_t) i16_output;
}

// call every 10ms: PI (velocity form) on the battery current, output is an absolute duty cycle (9 bit).
// Integral part per 100 ms is the same as the original integrator, the proportional part damps it.
uint16_t motor_current_controller (void)
{
  int16_t i16_error;
  int16_t i16_motor_current;
  int16_t i16_delta;
  int16_t i16_output;

  i16_motor_current = i16_motor_current_filtered_10b;
  if (i16_motor_current < 0) { i16_motor_current = 0; } // regen is not controlled here

  i16_error = ((int16_t) ui16_target_current_10b) - i16_motor_current;
  if (i16_error > 400) { i16_error = 400; }
  if (i16_error < -400) { i16_error = -400; }

  // in 1/16 duty cycle steps: I = 2 * error (= 10/32 per 100 ms in 8 bit duty), P = 16 * change of error
  i16_current_pi_fraction += (i16_error * 2) + ((i16_error - i16_current_error_last) * 16);
  i16_current_error_last = i16_error;
  i16_delta = i16_current_pi_fraction / 16;
  i16_current_pi_fraction -= i16_delta * 16;

  if (ui16_target_current_10b == 0) { i16_current_pi_fraction = 0; }

  i16_output = (int16_t) ui16_duty_cycle + i16_delta;
  if (i16_output > PWM_DUTY_MAX) i16_output = PWM_DUTY_MAX;
  if (i16_output < 0) i16_output = 0;
  return (uint16_t) i16_output;
}

void do_battery_voltage_protection (void)
{
  // low pass filter the voltage readed value, to avoid possible fast spikes/noise
  ui16_adc_battery_voltage_accumulated -= ui16_adc_battery_voltage_accumulated >> 6;
  ui16_adc_battery_voltage_accumulated += ((uint16_t) ui8_adc_read_battery_voltage ());
  ui8_adc_battery_voltage_filtered = ui16_adc_battery_voltage_accumulated >> 6;

  if (ui8_adc_battery_voltage_filtered > kt_settings.ui8_adc_battery_voltage_max)
  {
    if (settings_flag (KT_FLAG_OVER_VOLTAGE_PROTECTION))
    {
      // motor stops and the display shows the over voltage error (it used to show "battery empty")
      motor_controller_set_state (MOTOR_CONTROLLER_STATE_OVER_VOLTAGE);
      motor_disable_PWM ();
      motor_controller_set_error (MOTOR_CONTROLLER_ERROR_OVER_VOLTAGE);
    }
  }
  else if (motor_controller_state_is_set (MOTOR_CONTROLLER_STATE_OVER_VOLTAGE) &&
           (ui8_adc_battery_voltage_filtered < (kt_settings.ui8_adc_battery_voltage_max - 3))) // ~0.8 V hysteresis
  {
    // voltage is back to normal: restart from zero duty cycle
    ui16_duty_cycle = 0;
    ui16_duty_cycle_target = 0;
    motor_controller_reset_state (MOTOR_CONTROLLER_STATE_OVER_VOLTAGE);
    motor_controller_clear_error ();
  }
  else if (ui8_adc_battery_voltage_filtered < kt_settings.ui8_adc_battery_voltage_min)
  {
    // motor will stop and battery symbol on LCD will be empty and flashing
    motor_controller_set_state (MOTOR_CONTROLLER_STATE_UNDER_VOLTAGE);
    motor_disable_PWM ();
    motor_controller_set_error (MOTOR_CONTROLLER_ERROR_91_BATTERY_UNDER_VOLTAGE);
  }
}

uint8_t motor_get_ADC_battery_voltage_filtered (void)
{
  return ui8_adc_battery_voltage_filtered;
}

void motor_controller_set_error (uint8_t error)
{
  ui8_motor_controller_error = error;
}

void motor_controller_clear_error (void)
{
  ui8_motor_controller_error = 0;
}

uint8_t motor_controller_get_error (void)
{
  return ui8_motor_controller_error;
}

void motor_set_pwm_duty_cycle (uint16_t ui16_value)
{
  if (ui16_value > PWM_DUTY_MAX) { ui16_value = PWM_DUTY_MAX; }
  ui16_duty_cycle = ui16_value;
}

void do_motor_state_machine (void)
{
  uint16_t ui16_motor_speed_erps;

  ui16_motor_speed_erps = ui16_motor_get_motor_speed_erps ();

  switch (ui8_motor_state)
  {
    case MOTOR_STATE_STOP:
    if ((ui16_motor_speed_erps < 5) && (!ebike_app_is_throttle_released ()))
    {
      ui8_motor_startup_counter = 0;
      ui8_motor_state = MOTOR_STATE_STARTUP;
    }
    else if (ui16_motor_speed_erps > 4)
    {
      ui8_motor_state = MOTOR_STATE_RUNNING;
    }
    break;

    case MOTOR_STATE_STARTUP:
    if (ui8_motor_startup_counter++ > 19) // 20 * 100ms; 2 seconds max time
    {
      motor_controller_set_state (MOTOR_CONTROLLER_STATE_MOTOR_BLOCKED);
      motor_disable_PWM ();
      ebike_app_cruise_control_stop ();
      ui8_motor_state = MOTOR_STATE_COOL;
      ui8_motor_startup_counter = 0;
    }

    if (ui16_motor_speed_erps > 4)
    {
      ui8_motor_state = MOTOR_STATE_RUNNING;
    }
    break;

    case MOTOR_STATE_RUNNING:
    break;

    // wait for the power mosfets cool down and for user release the throttle
    case MOTOR_STATE_COOL:
    if (ui8_motor_startup_counter++ > 10) // 1 second timeout
    {
      ui8_motor_startup_counter = 11;

      if (ebike_app_is_throttle_released ())
      {
        ui16_duty_cycle = 0;
        ui16_duty_cycle_target = 0;
        motor_controller_reset_state (MOTOR_CONTROLLER_STATE_MOTOR_BLOCKED);
        ui8_motor_state = MOTOR_STATE_STOP;
      }
    }
    break;
  }
}

void calc_motor_current_filtered (void)
{
  // low pass filter the current readed value, to avoid possible fast spikes/noise
  ui16_adc_motor_current_accumulated_10b -= ui16_adc_motor_current_accumulated_10b >> 4;
  ui16_adc_motor_current_accumulated_10b += ui16_adc_read_motor_total_current_10b ();
  ui16_adc_motor_current_filtered_10b = ui16_adc_motor_current_accumulated_10b >> 4;
  i16_motor_current_filtered_10b = (int16_t) ui16_adc_motor_current_filtered_10b - (int16_t) ui16_motor_total_current_offset_10b;
}

// motor overcurrent interrupt
void EXTI_PORTD_IRQHandler(void) __interrupt(EXTI_PORTD_IRQHANDLER)
{
  // motor will stop and error symbol on LCD will be shown
  motor_controller_set_state (MOTOR_CONTROLLER_STATE_OVER_CURRENT);
  motor_disable_PWM ();
  motor_controller_set_error (MOTOR_CONTROLLER_ERROR_06_SHORT_CIRCUIT);
}
