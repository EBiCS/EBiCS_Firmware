/*
 * sensorless.c
 *
 * Sensorless rotor angle estimation, ported from the EBiCS Sensorless_VESC branch.
 * - flux observer (fixed point, from observer_update() of the branch)
 * - open loop start: the voltage vector is rotated until the observer can take over
 * - electrical speed from the observer angle, fed into the existing speed variables so
 *   display speed, speed limit and standstill detection keep working without hall sensors
 *
 * This file is part of EBiCS, GPL v3 or later.
 */

#include "config.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include <arm_math.h>

#ifdef SENSORLESS

#include "sensorless.h"

// voltage scaling for the observer (ADC * 25.6 mV / sqrt(3) = 15, <<8), see the Sensorless_VESC branch
#ifndef OBS_CAL_V
#define OBS_CAL_V (15LL<<8)
#endif
// angle correction for the current filter delay
#ifndef FILTER_DELAY
#define FILTER_DELAY (59652323L<<4)
#endif
// total angle offset between observer and rotor (branch: atan offset 1550960412 + SPEC_ANGLE -1312351118)
#ifndef OBS_ANGLE_OFFSET
#define OBS_ANGLE_OFFSET 238609294L
#endif
#define STARTUP_STEP     2684354     // open loop angle step per FOC cycle
#define STARTUP_CYCLES   4000        // FOC cycles of open loop before the observer takes over
#define STARTUP_UD_MAX   250         // max open loop voltage (u_d)
#define STALL_CYCLES     10000       // FOC cycles without a full electric revolution = stalled

extern uint16_t uint16_full_rotation_counter;
extern uint16_t uint16_half_rotation_counter;
extern uint32_t uint32_tics_filtered;
extern uint16_t ui16_timertics;
extern volatile uint8_t ui8_SPEED_control_flag;
extern volatile uint16_t adcData[];

q31_t q31_teta_obs = 0;
volatile uint8_t ui8_sensorless_running = 0;
volatile q31_t q31_erps_filtered = STALL_CYCLES;

static q31_t startup_counter = 0;
static q31_t erps_counter = STALL_CYCLES;
static q31_t erps_cumulated = STALL_CYCLES << 4;
static q31_t angle_old = 0;
static q31_t sin_delay = 0, cos_delay = 2147483647;
static long long x1 = 0, x2 = 0;

// atan(i/100) for i = 0..100, 45 deg = 16384
static const uint16_t LUT_atan[101] = {
	0, 209, 417, 626, 834, 1042, 1250, 1458, 1665, 1872, 2079, 2285,
	2491, 2697, 2902, 3106, 3310, 3513, 3715, 3917, 4118, 4318, 4517, 4716,
	4914, 5110, 5306, 5501, 5695, 5888, 6080, 6271, 6461, 6649, 6837, 7023,
	7209, 7393, 7576, 7757, 7938, 8117, 8295, 8472, 8647, 8821, 8994, 9165,
	9336, 9504, 9672, 9838, 10003, 10167, 10329, 10490, 10649, 10807, 10964, 11119,
	11274, 11426, 11578, 11728, 11876, 12024, 12170, 12314, 12458, 12600, 12740, 12880,
	13018, 13154, 13290, 13424, 13557, 13688, 13819, 13948, 14076, 14202, 14328, 14452,
	14575, 14696, 14817, 14936, 15054, 15171, 15287, 15402, 15515, 15628, 15739, 15849,
	15958, 16066, 16173, 16279, 16383 };

static int32_t lut_atan_ratio(uint32_t num, uint32_t den)   // atan(num/den), num <= den, 90 deg = 32768
{
	uint32_t r = (num * 1000U) / den;   // 0..1000, inputs are scaled below 2^21
	uint32_t idx = r / 10, frac = r % 10;
	if (idx >= 100) return LUT_atan[100];
	return LUT_atan[idx] + ((LUT_atan[idx + 1] - LUT_atan[idx]) * frac) / 10;
}

// robust atan2 (the branch version overflowed for large inputs), result: +-2^31 = +-180 deg
static q31_t atan2_q31(long long y, long long x)
{
	long long ax = x < 0 ? -x : x;
	long long ay = y < 0 ? -y : y;
	if (!ax && !ay) return 0;
	while (ax >= (1LL << 21) || ay >= (1LL << 21)) { ax >>= 1; ay >>= 1; }
	if (!ax && !ay) return 0;

	int32_t ang;   // 0..65536 = 0..180 deg
	if (ay <= ax) ang = lut_atan_ratio((uint32_t)ay, (uint32_t)ax);
	else          ang = 32768 - lut_atan_ratio((uint32_t)ax, (uint32_t)ay);
	if (x < 0) ang = 65536 - ang;

	uint32_t q = (uint32_t)ang << 15;       // 65536<<15 wraps to -180 deg, which is the same angle
	return (y < 0) ? -(q31_t)q : (q31_t)q;
}

static int truncate_abs(long long *number, long long max)
{
	if (*number > max)  { *number = max;  return 1; }
	if (*number < -max) { *number = -max; return 1; }
	return 0;
}

// See http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
static void observer_update(long long v_alpha, long long v_beta, long long i_alpha, long long i_beta,
                            long long *e_alpha, long long *e_beta)
{
	const long long L = (3LL * RS.obs_inductance) >> 1;
	const long long R = (3LL * RS.obs_resistance) >> 1;
	const long long lambda = RS.obs_flux_linkage;
	const long long lambda_2 = lambda * lambda;
	const long long dT = 13LL;

	const long long L_ia = (L * i_alpha) >> 16;
	const long long L_ib = (L * i_beta) >> 16;
	const long long R_ia = (R * i_alpha) >> 9;
	const long long R_ib = (R * i_beta) >> 9;

	*e_alpha = x1 - L_ia;
	*e_beta  = x2 - L_ib;
	long long err = lambda_2 - ((*e_alpha * *e_alpha) + (*e_beta * *e_beta));
	truncate_abs(&err, lambda_2);

	long long x1_dot = -R_ia + v_alpha + ((*e_alpha * err) >> RS.obs_gamma);
	long long x2_dot = -R_ib + v_beta  + ((*e_beta * err) >> RS.obs_gamma);
	x1 += x1_dot >> dT;
	x2 += x2_dot >> dT;

	*e_alpha = x1 - L_ia;
	*e_beta  = x2 - L_ib;
}

void sensorless_reset(void)
{
	ui8_sensorless_running = 0;
	startup_counter = 0;
	erps_counter = STALL_CYCLES;
	x1 = 0;
	x2 = 0;
}

void sensorless_startup(MotorState_t* MS_FOC, int16_t target)
{
	if (ui8_sensorless_running) return;
	if (target > 20 || target < -20) {
		// open loop: rotating voltage vector, slowly increasing
		q31_t ud = startup_counter >> 4;
		MS_FOC->u_d = ud > STARTUP_UD_MAX ? STARTUP_UD_MAX : ud;
		q31_teta_obs += (target > 0) ? STARTUP_STEP : -STARTUP_STEP;
		if (++startup_counter > STARTUP_CYCLES) {
			ui8_sensorless_running = 1;
			startup_counter = 0;
		}
	}
}

void sensorless_observe(q31_t u_alpha, q31_t u_beta, q31_t i_alpha, q31_t i_beta, int16_t target)
{
	q31_t ia_c, ib_c;
	long long e_a, e_b;

	// correct the phase shift of the filtered currents (rotate by the delay angle)
	arm_park_q31(i_alpha, i_beta, &ia_c, &ib_c, sin_delay, cos_delay);

	long long vbat = adcData[0];
	observer_update(((long long)u_alpha * vbat * OBS_CAL_V) >> 11,
	                ((long long)(-u_beta) * vbat * OBS_CAL_V) >> 11,
	                (long long)(-ia_c) * (long long)CAL_I,
	                (long long)(-ib_c) * (long long)CAL_I,
	                &e_a, &e_b);

	if (ui8_sensorless_running) {
		q31_teta_obs = atan2_q31(e_a, -e_b) + RS.obs_angle_offset;
	}

	// electrical speed: count FOC cycles per electric revolution (angle crossing +-180 deg)
	if (erps_counter < STALL_CYCLES) erps_counter++;
	else {
		ui8_sensorless_running = 0;   // stalled: start again with the open loop
		q31_erps_filtered = STALL_CYCLES;
	}
	uint8_t wrapped = (angle_old > (1 << 25) && q31_teta_obs < -(1 << 25)) ||   // forward
	                  (angle_old < -(1 << 25) && q31_teta_obs > (1 << 25));     // backward
	if (wrapped && erps_counter > 15) {
		erps_cumulated -= erps_cumulated >> 4;
		erps_cumulated += erps_counter;
		q31_erps_filtered = erps_cumulated >> 4;
		erps_counter = 0;
		// feed the hall based speed variables: 6 hall events per electric revolution,
		// 500 kHz timer2 tics, FOC runs at about 15.78 kHz -> tics per hall event = cycles * 5.24
		ui16_timertics = (uint16_t)((q31_erps_filtered * 21) >> 2);
		uint32_tics_filtered = (uint32_t)ui16_timertics << 3;
		ui8_SPEED_control_flag = 1;
		uint16_full_rotation_counter = 0;   // motor is turning: no standstill switch-off
		uint16_half_rotation_counter = 0;
	}
	angle_old = q31_teta_obs;
	(void)target;
}

void sensorless_slow(void)
{
	arm_sin_cos_q31(FILTER_DELAY / (q31_erps_filtered + 1), &sin_delay, &cos_delay);
}

#endif // SENSORLESS
