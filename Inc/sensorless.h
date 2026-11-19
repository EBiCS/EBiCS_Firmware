/*
 * sensorless.h
 *
 * Sensorless rotor angle estimation, ported from the EBiCS Sensorless_VESC branch
 * (flux observer after Lee/Hong/Nam/Ortega/Praly/Astolfi 2010, as used in the VESC).
 * Enabled with #define SENSORLESS in config.h. EXPERIMENTAL: the motor parameters
 * INDUCTANCE, RESISTANCE, FLUX_LINKAGE, GAMMA and OBS_ANGLE_OFFSET must match the motor.
 *
 * This file is part of EBiCS, GPL v3 or later.
 */

#ifndef SENSORLESS_H_
#define SENSORLESS_H_

#include <arm_math.h>
#include "main.h"

extern q31_t q31_teta_obs;                    // estimated rotor angle, pass to FOC_calculation
extern volatile uint8_t ui8_sensorless_running; // 0 = open loop start / stopped, 1 = observer in control
extern volatile q31_t q31_erps_filtered;      // FOC cycles per electrical revolution (filtered)

void sensorless_reset(void);                                         // call when PWM is switched on
void sensorless_startup(MotorState_t* MS_FOC, int16_t target);       // call after the PI control, before inverse Park
void sensorless_observe(q31_t u_alpha, q31_t u_beta, q31_t i_alpha, q31_t i_beta, int16_t target); // after inverse Park
int32_t sensorless_q_setpoint(int32_t setpoint);                     // q current setpoint during start / hand-over
void sensorless_slow(void);                                          // call from the slow loop

#endif /* SENSORLESS_H_ */
