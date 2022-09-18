/*
 * hubsensor.c
 *
 *  Created on: 18.09.2022
 *      Author: gaswerke
 */


#include "hubsensor.h"
#include "stm32f1xx_hal.h"
#include "print.h"

uint8_t UART3_RxBuff[14]; //Hub sends blocks of 7 bytes, one complete message must be within 14 bytes.
UART_HandleTypeDef huart3;

void Hubsensor_Init (Hubsensor_t* HS_data){
    if (HAL_UART_Receive_DMA(&huart3, (uint8_t *)UART3_RxBuff,14) != HAL_OK)
     {
 	   Error_Handler();
     }
}

void Hubsensor_Service (Hubsensor_t* HS_data){
	uint8_t i;
	i=13;
	while (i>0&&UART3_RxBuff[i]!=0x00 && UART3_RxBuff[i-1]!=0xFF){
		i--;
	}
	//printf_("%d ,%d  %d\n ", i, (UART3_RxBuff[i-6]+UART3_RxBuff[i-5]+UART3_RxBuff[i-4]+UART3_RxBuff[i-3])%256,UART3_RxBuff[i-2]+1);
	if ((UART3_RxBuff[i-6]+UART3_RxBuff[i-5]+UART3_RxBuff[i-4]+UART3_RxBuff[i-3])%256 == UART3_RxBuff[i-2]+1){
		HS_data->HS_Overtemperature = UART3_RxBuff[i-6]>>7;
		HS_data->HS_Pedalposition = UART3_RxBuff[i-5]&127;
		HS_data->HS_Pedals_turning = UART3_RxBuff[i-5]>>7;
		HS_data->HS_Torque = UART3_RxBuff[i-6]&127;
		HS_data->HS_Wheel_turning = UART3_RxBuff[i-4]>>7;
		HS_data->HS_Wheeltime = ((UART3_RxBuff[i-4]&127)<<8)&UART3_RxBuff[i-3];
	}
}
