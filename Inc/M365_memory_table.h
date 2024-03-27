/*
 * M365_memory_table.h
 *
 *  Created on: 19.12.2021
 *      Author: Stancecoke
 *      taken from https://github.com/rascafr/9-analyzer/blob/master/definitions.js
 *      and
 */

#ifndef INC_M365_MEMORY_TABLE_H_
#define INC_M365_MEMORY_TABLE_H_

typedef struct __attribute__ ((__packed__)) {

	uint16_t magic[13];
	uint16_t currA;
	uint16_t currB;
	uint16_t currC;
	char scooter_serial[14];
	uint16_t scooter_pin[3];
	uint16_t ESC_version;
	uint16_t error_code_1;
	uint16_t alarm_code_1;
	uint32_t ESC_status_1;
	uint16_t ride_mode;
	uint16_t capacity_battery_1;
	uint16_t capacity_battery_2;
	uint32_t battery_level_1;
	uint16_t recent_remaining_range;
	uint16_t predicted_remaining_range;
	uint16_t recent_speed_1[3];
	uint16_t total_milage[6];
	uint16_t recent_milage[3];
	uint32_t total_operation_time;
	uint16_t total_riding_time[6];
	uint16_t single_operating_time_1;
	uint16_t single_riding_time_1[3];
	uint16_t frame_temperature;
	uint16_t battery1_temperature;
	uint16_t battery2_temperature;
	uint16_t MOSFET_temperature[6];
	uint16_t ESC_supply_voltage;
	uint16_t BMS_supply_voltage;
	uint16_t BMS_current[7];
	uint16_t external_battery_temperature[3];
	uint16_t motor_phase_current[18];
	uint16_t average_speed;
	uint16_t external_battery_version;
	uint16_t internal_battery_version;
	uint16_t BLE_version[8];
	uint16_t lock_command;
	uint16_t unlock_command;
	uint16_t speed_limit;
	uint16_t speed_limit_normal;
	uint16_t speed_limit_eco;
	uint16_t operation_mode;
	uint16_t start_stop;
	uint16_t reboot;
	uint16_t power_off;
	uint16_t unknown;
	uint16_t regen_level;
	uint16_t cruise_control;
	uint16_t tail_light[52];
	uint16_t error_code_2;
	uint16_t alarm_code_2;
	uint16_t ESC_status_2;
	uint16_t capacity_battery_1_2;
	uint16_t battery_2;
	uint16_t recent_speed_2;
	uint16_t average_speed_2;
	uint32_t total_milage_2;
	uint16_t recent_milage_2;
	uint16_t single_operation_time_2;
	uint16_t frame_temperature_2;
	uint16_t recent_speed_limit_2;
	uint16_t scooter_power;
	uint16_t previous_alarm_code;
	uint16_t predicted_remaining_range_2[7];
	uint32_t display_mode_lamp_strip;
	uint32_t colour_strip_1;
	uint32_t colour_strip_2;
	uint32_t colour_strip_3;
	uint32_t colour_strip_4;

} M365_menory_table_t;

#endif /* INC_M365_MEMORY_TABLE_H_ */
