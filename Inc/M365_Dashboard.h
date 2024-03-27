/*
 * M365_Dashboard.h
 *
 *  Created on: Nov 27, 2021
 *      Author: stancecoke
 */

#ifndef INC_M365_DASHBOARD_H_
#define INC_M365_DASHBOARD_H_


void M365Dashboard_init();

void search_DashboardMessage(MotorState_t *MS, MotorParams_t *MP,UART_HandleTypeDef huart1);
void send_DashboardMessage(uint8_t page, MotorState_t *MS, MotorParams_t *MP);
void process_DashboardMessage(MotorState_t *MS, MotorParams_t *MP, uint8_t *message, uint8_t length, UART_HandleTypeDef huart1 );
void addCRC(uint8_t * message, uint8_t size);
int16_t checkCRC(uint8_t * message, uint8_t size);



#endif /* INC_M365_DASHBOARD_H_ */
