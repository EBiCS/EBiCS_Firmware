/*
 * settings.h - runtime settings block
 *
 * The numeric bike settings (currents, voltages, speed limit, wheel, gearing, PAS, ramps, temperatures ...)
 * are read at boot from a 256 byte block at a fixed flash address. The EBiCS Setup Android app writes this
 * block when it flashes a prebuilt firmware, so the phone does not need a compiler.
 *
 * If the block is missing (erased flash), has a wrong magic/version/size, a bad CRC or implausible values,
 * the firmware uses the compile-time values from config.h (written by the desktop setup app or by hand).
 * The desktop build puts an all-zero block there, so an old phone block never overrides a fresh config.h.
 *
 * Layout (little endian, version 1, 256 bytes). Keep in sync with android_app/lib/firmware/settings_block.dart
 * and setup_app/ebics_core.py (SETTINGS_FIELDS).
 *
 * Part of EBiCS, GPL v3 or later.
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <stdint.h>

#define EBICS_SETTINGS_ADDR     0x08007700UL   // page 29, last 256 bytes before the emulated EEPROM (pages 30+31)
#define EBICS_SETTINGS_MAGIC    0xEB1C5E77UL
#define EBICS_SETTINGS_VERSION  1
#define EBICS_SETTINGS_SIZE     256

// flags
#define SF_THROTTLE        (1UL<<0)   // throttle connected (THROTTLE_OVERRIDE)
#define SF_LEGAL           (1UL<<1)   // EU legal mode (LEGALFLAG)
#define SF_TORQUE_SENSOR   (1UL<<2)   // torque sensor mode (TS_MODE)
#define SF_TORQUE_ON_AD1   (1UL<<3)   // torque signal on AD1 (TQONAD1)
#define SF_DIRDET          (1UL<<4)   // stop on backpedalling (DIRDET)
#define SF_ADC_BRAKE       (1UL<<5)   // proportional regen on AD2 (ADC_BRAKE)
#define SF_SPEED_EXTERNAL  (1UL<<6)   // speed from an external sensor or the speed wire (SPEEDSOURCE == EXTERNAL)
#define SF_SPEED_FROM_WIRE (1UL<<7)   // speed pulses from the analog speed/temperature wire (SPEED_FROM_ADC)
#define SF_TEMP_FROM_WIRE  (1UL<<8)   // motor NTC on the speed/temperature wire (R_TEMP_PULLUP)

typedef struct {
	uint32_t magic;                 //   0  EBICS_SETTINGS_MAGIC
	uint16_t version;               //   4  EBICS_SETTINGS_VERSION
	uint16_t size;                  //   6  EBICS_SETTINGS_SIZE
	uint32_t flags;                 //   8  SF_*
	int32_t ph_current_max;         //  12  internal current units (PH_CURRENT_MAX)
	int32_t battery_current_max;    //  16  mA
	int32_t regen_current;          //  20  internal current units
	int32_t regen_current_max;      //  24  mA
	int32_t battery_voltage_min;    //  28  mV, cutoff
	int32_t battery_voltage_max;    //  32  mV, full charge
	int32_t voltage_ramp_mv;        //  36  mV
	int32_t battery_level[5];       //  40  battery bars, mV*10 (BATTERY_LEVEL_1..5)
	int32_t speed_limit;            //  60  km/h
	int32_t wheel_circumference;    //  64  mm
	int32_t gear_ratio;             //  68  pole pairs x gear ratio
	int32_t pas_imp_per_turn;       //  72
	int32_t pulses_per_revolution;  //  76  external speed sensor magnets
	int32_t pushassist_current;     //  80  internal current units
	int32_t ts_coef;                //  84
	int32_t pas_timeout;            //  88  ticks
	int32_t ramp_end;               //  92  ticks
	int32_t throttle_offset;        //  96  ADC
	int32_t throttle_max;           // 100  ADC
	int32_t motor_temp_threshold;   // 104  deg C
	int32_t motor_temp_max;         // 108  deg C
	int32_t frac_high;              // 112
	int32_t frac_low;               // 116
	int32_t reverse;                // 120  1 normal, -1 reverse
	int32_t speed_temp_adc;         // 124  adcData index of the speed/temperature wire: 6 = AD1, 5 = AD2
	int32_t speed_temp_threshold;   // 128  ADC
	int32_t ntc_pullup_ohm;         // 132
	int32_t obs_inductance;         // 136  sensorless observer (INDUCTANCE)
	int32_t obs_resistance;         // 140  (RESISTANCE)
	int32_t obs_flux_linkage;       // 144  (FLUX_LINKAGE)
	int32_t obs_gamma;              // 148  (GAMMA, shift)
	int32_t obs_angle_offset;       // 152  q31 (OBS_ANGLE_OFFSET)
	uint32_t reserved[24];          // 156  0
	uint32_t crc;                   // 252  CRC-32 (IEEE, like zlib) of bytes 0..251
} ebics_settings_t;

extern ebics_settings_t RS;                      // the settings in use (RAM copy)
extern volatile uint8_t ui8_settings_source;     // 0 = config.h, 1 = flash block, 2 = flash block rejected (config.h used)

void settings_load(void);
uint32_t settings_crc32(const uint8_t *data, uint32_t len);

#define RS_FLAG(f) ((RS.flags & (f)) != 0)

#endif /* SETTINGS_H_ */
