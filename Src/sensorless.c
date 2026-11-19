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
#define STARTUP_RAMP     1000        // FOC cycles to ramp the open loop current up
#define HANDOVER_CYCLES  1000        // FOC cycles to fade from open loop current to the requested current
#define STALL_CYCLES     10000       // FOC cycles without a full electric revolution = stalled
// observer time step = one FOC period: x += x_dot * Ts, Ts = 1/15779 s = OBS_DT_K / 2^28
// (the branch used x_dot>>13 = 1/8192 s, 1.93x too long: the angle was 45..95 deg off and got worse with current)
#define OBS_DT_K         17012LL

extern uint16_t uint16_full_rotation_counter;
extern uint16_t uint16_half_rotation_counter;
extern uint32_t uint32_tics_filtered;
extern uint16_t ui16_timertics;
extern volatile uint8_t ui8_SPEED_control_flag;
extern volatile uint16_t adcData[];
extern PI_control_t PI_iq;
extern PI_control_t PI_id;
extern q31_t q31_i_q_fil;
extern q31_t q31_i_d_fil;

q31_t q31_teta_obs = 0;
volatile uint8_t ui8_sensorless_running = 0;
volatile q31_t q31_erps_filtered = STALL_CYCLES;

static q31_t startup_counter = 0;
static q31_t handover_counter = HANDOVER_CYCLES;
static int32_t i_start = 0;                 // open loop current (PI units)
static int32_t handover_d0 = 0, handover_q0 = 0; // current setpoints at the hand-over, in the observer frame
static q31_t q31_obs_angle = 0;             // observer angle, computed every cycle (also during the open loop)
static q31_t erps_counter = 0;
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
	(void)dT;
	x1 += (x1_dot * OBS_DT_K) >> 28;
	x2 += (x2_dot * OBS_DT_K) >> 28;

	*e_alpha = x1 - L_ia;
	*e_beta  = x2 - L_ib;
}

void sensorless_reset(void)
{
	ui8_sensorless_running = 0;
	startup_counter = 0;
	handover_counter = HANDOVER_CYCLES;
	erps_counter = 0;              // was STALL_CYCLES: the first call then reported a stall at once
	x1 = 0;
	x2 = 0;
}

// a vector given in the firmware dq frame at angle 'from' expressed in the frame at angle 'to'
// (same convention as the inverse Park call in FOC_calculation: alpha = d*c + q*s, beta = q*c - d*s)
static void change_frame(q31_t d, q31_t q, q31_t from, q31_t to, q31_t *d_new, q31_t *q_new)
{
	q31_t s1, c1, s2, c2;
	arm_sin_cos_q31(from, &s1, &c1);
	arm_sin_cos_q31(to, &s2, &c2);
	long long a = (((long long)d * c1) >> 31) + (((long long)q * s1) >> 31);
	long long b = (((long long)q * c1) >> 31) - (((long long)d * s1) >> 31);
	*d_new = (q31_t)(((a * c2) >> 31) - ((b * s2) >> 31));
	*q_new = (q31_t)(((a * s2) >> 31) + ((b * c2) >> 31));
}

void sensorless_startup(MotorState_t* MS_FOC, int16_t target)
{
	if (ui8_sensorless_running) {
		// fade from the open loop current to the requested current, see sensorless_q_setpoint()
		if (handover_counter < HANDOVER_CYCLES) {
			handover_counter++;
			MS_FOC->i_d_setpoint = (handover_d0 * (HANDOVER_CYCLES - handover_counter)) / HANDOVER_CYCLES;
		}
		else MS_FOC->i_d_setpoint = 0;
		return;
	}
	if (target > 20 || target < -20) {
		// current controlled open loop (I/f, like the hall autodetect): rotate the angle, regulate i_d,
		// q current 0. The old fixed voltage ramp gave motor dependent currents.
		int32_t t = target > 0 ? target : -target;
		i_start = t < RS.ph_current_max ? t : RS.ph_current_max;
		MS_FOC->i_d_setpoint = (startup_counter < STARTUP_RAMP) ? (i_start * startup_counter) / STARTUP_RAMP : i_start;
		q31_teta_obs += (target > 0) ? STARTUP_STEP : -STARTUP_STEP;
		if (++startup_counter > STARTUP_CYCLES) {
			// bumpless hand-over: express voltages, filtered currents and current setpoints in the observer
			// frame, so nothing jumps in the stator frame (without this: 100+ A current spike in the simulation)
			q31_t d, q;
			q31_t from = q31_teta_obs, to = q31_obs_angle;
			change_frame(MS_FOC->u_d, MS_FOC->u_q, from, to, &d, &q);
			PI_id.integral_part = PI_id.out = -(d << PI_id.shift);   // u_d = -PI_id output
			PI_iq.integral_part = PI_iq.out = q << PI_iq.shift;
			change_frame(q31_i_d_fil, q31_i_q_fil, from, to, &d, &q);
			q31_i_d_fil = d;
			q31_i_q_fil = q;
			change_frame(MS_FOC->i_d_setpoint, 0, from, to, &d, &q);
			handover_d0 = d;
			handover_q0 = q;
			handover_counter = 0;
			q31_teta_obs = to;
			ui8_sensorless_running = 1;
			startup_counter = 0;
		}
	}
}

int32_t sensorless_q_setpoint(int32_t setpoint)
{
	if (!ui8_sensorless_running) return 0;                       // open loop: q current 0
	if (handover_counter < HANDOVER_CYCLES)
		return (handover_q0 * (HANDOVER_CYCLES - handover_counter) + setpoint * handover_counter) / HANDOVER_CYCLES;
	return setpoint;
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

	q31_obs_angle = atan2_q31(e_a, -e_b) + RS.obs_angle_offset;
	if (ui8_sensorless_running) {
		q31_teta_obs = q31_obs_angle;
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
