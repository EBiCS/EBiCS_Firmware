/*
 * display_ebics.h
 *
 *  Created on: 11.11.2020
 *      Author: Gaswerke
 */

#ifndef DISPLAY_EBICS_H_
#define DISPLAY_EBICS_H_

void ebics_init();
void process_ant_page(MotorState_t* MS_U);
void send_ant_page(MotorState_t* MS_D);

typedef struct _ant_variables
{
  uint8_t ui8_assist_level;
  uint8_t ui8_light;
  uint8_t ui8_motor_characteristic;
  uint8_t ui8_wheel_size;
  uint8_t ui8_max_speed;
  uint8_t ui8_power_assist_control_mode;
  uint8_t ui8_controller_max_current;
} struc__ant_variables;

#endif /* DISPLAY_EBICS_H_ */
