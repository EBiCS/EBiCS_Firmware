/*
 * settings.c - runtime settings block, see settings.h
 *
 * Part of EBiCS, GPL v3 or later.
 */

#include <string.h>
#include "config.h"
#include "main.h"
#include "settings.h"

#ifndef OBS_ANGLE_OFFSET
#define OBS_ANGLE_OFFSET 238609294L
#endif
#ifndef INDUCTANCE
#define INDUCTANCE 2
#endif
#ifndef RESISTANCE
#define RESISTANCE 120
#endif
#ifndef FLUX_LINKAGE
#define FLUX_LINKAGE 2400
#endif
#ifndef GAMMA
#define GAMMA 13
#endif

typedef char settings_size_check[(sizeof(ebics_settings_t) == EBICS_SETTINGS_SIZE) ? 1 : -1];

// Placeholder in flash at EBICS_SETTINGS_ADDR (see the linker script). All zero = not valid, so a firmware
// installed from a desktop build always uses its config.h, even if a phone wrote a block there before.
__attribute__((section(".ebics_settings"), used))
const uint8_t ebics_settings_flash[EBICS_SETTINGS_SIZE] = {0};

ebics_settings_t RS;
volatile uint8_t ui8_settings_source = 0;

static const ebics_settings_t settings_default = {
	.magic = EBICS_SETTINGS_MAGIC,
	.version = EBICS_SETTINGS_VERSION,
	.size = EBICS_SETTINGS_SIZE,
	.flags = 0
#ifdef THROTTLE_OVERRIDE
		| SF_THROTTLE
#endif
#ifdef LEGALFLAG
		| SF_LEGAL
#endif
#ifdef TS_MODE
		| SF_TORQUE_SENSOR
#endif
#ifdef TQONAD1
		| SF_TORQUE_ON_AD1
#endif
#ifdef DIRDET
		| SF_DIRDET
#endif
#ifdef ADC_BRAKE
		| SF_ADC_BRAKE
#endif
#if (SPEEDSOURCE == EXTERNAL)
		| SF_SPEED_EXTERNAL
#endif
#ifdef SPEED_FROM_ADC
		| SF_SPEED_FROM_WIRE
#endif
#if (R_TEMP_PULLUP)
		| SF_TEMP_FROM_WIRE
#endif
		,
	.ph_current_max = PH_CURRENT_MAX,
	.battery_current_max = BATTERYCURRENT_MAX,
	.regen_current = REGEN_CURRENT,
	.regen_current_max = REGEN_CURRENT_MAX,
	.battery_voltage_min = BATTERYVOLTAGE_MIN,
	.battery_voltage_max = BATTERYVOLTAGE_MAX,
	.voltage_ramp_mv = VOLTAGE_RAMP_MV,
	.battery_level = {BATTERY_LEVEL_1, BATTERY_LEVEL_2, BATTERY_LEVEL_3, BATTERY_LEVEL_4, BATTERY_LEVEL_5},
	.speed_limit = SPEEDLIMIT,
	.wheel_circumference = WHEEL_CIRCUMFERENCE,
	.gear_ratio = GEAR_RATIO,
	.pas_imp_per_turn = PAS_IMP_PER_TURN,
	.pulses_per_revolution = PULSES_PER_REVOLUTION,
	.pushassist_current = PUSHASSIST_CURRENT,
	.ts_coef = TS_COEF,
	.pas_timeout = PAS_TIMEOUT,
	.ramp_end = RAMP_END,
	.throttle_offset = THROTTLE_OFFSET,
	.throttle_max = THROTTLE_MAX,
	.motor_temp_threshold = MOTOR_TEMPERATURE_THRESHOLD,
	.motor_temp_max = MOTOR_TEMPERATURE_MAX,
	.frac_high = FRAC_HIGH,
	.frac_low = FRAC_LOW,
	.reverse = REVERSE,
	.speed_temp_adc = SPEED_TEMP_ADC,
	.speed_temp_threshold = SPEED_TEMP_THRESHOLD,
	.ntc_pullup_ohm = R_TEMP_PULLUP,
	.obs_inductance = INDUCTANCE,
	.obs_resistance = RESISTANCE,
	.obs_flux_linkage = FLUX_LINKAGE,
	.obs_gamma = GAMMA,
	.obs_angle_offset = (int32_t)(OBS_ANGLE_OFFSET),
};

uint32_t settings_crc32(const uint8_t *data, uint32_t len)
{
	uint32_t crc = 0xFFFFFFFFUL;
	while (len--) {
		crc ^= *data++;
		for (uint8_t b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1)));
	}
	return ~crc;
}

// values that would divide by zero or make no sense: reject the whole block
static uint8_t settings_plausible(const ebics_settings_t *s)
{
	if (s->ph_current_max <= 0 || s->ph_current_max > 4000) return 0;
	if (s->battery_current_max <= 0 || s->battery_current_max > 60000) return 0;
	if (s->regen_current < 0 || s->regen_current_max < 0) return 0;
	if (s->battery_voltage_min < 10000 || s->battery_voltage_max > 100000) return 0;
	if (s->battery_voltage_max <= s->battery_voltage_min + 1000) return 0;
	if (s->voltage_ramp_mv < 100 || s->voltage_ramp_mv * 2 >= s->battery_voltage_max - s->battery_voltage_min) return 0;
	if (s->speed_limit < 1 || s->speed_limit > 100) return 0;
	if (s->wheel_circumference < 500 || s->wheel_circumference > 3500) return 0;
	if (s->gear_ratio < 1 || s->gear_ratio > 1000) return 0;
	if (s->pas_imp_per_turn < 1 || s->pas_imp_per_turn > 64) return 0;
	if (s->pulses_per_revolution < 1 || s->pulses_per_revolution > 64) return 0;
	if (s->pas_timeout <= s->ramp_end || s->ramp_end < 1) return 0;
	if (s->throttle_max <= 0 || s->throttle_max > 4095) return 0;
	if (s->motor_temp_max <= s->motor_temp_threshold) return 0;
	if (s->reverse != 1 && s->reverse != -1) return 0;
	if ((s->flags & (SF_SPEED_FROM_WIRE | SF_TEMP_FROM_WIRE)) && s->speed_temp_adc != 5 && s->speed_temp_adc != 6) return 0;
	if ((s->flags & SF_TEMP_FROM_WIRE) && s->ntc_pullup_ohm < 1) return 0;
	if (s->obs_gamma < 0 || s->obs_gamma > 30) return 0;
	return 1;
}

void settings_load(void)
{
	const ebics_settings_t *f = (const ebics_settings_t *) EBICS_SETTINGS_ADDR;
	RS = settings_default;
	ui8_settings_source = 0;
	if (f->magic != EBICS_SETTINGS_MAGIC) return;   // no block (erased flash or desktop build)
	if (f->version == EBICS_SETTINGS_VERSION && f->size == EBICS_SETTINGS_SIZE &&
			f->crc == settings_crc32((const uint8_t *) f, EBICS_SETTINGS_SIZE - 4) && settings_plausible(f)) {
		memcpy(&RS, f, sizeof(RS));
		ui8_settings_source = 1;
	}
	else ui8_settings_source = 2;
}
