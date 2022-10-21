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

uint8_t torque_offset = 150;
uint16_t torque_cumulated = 0;

void Hubsensor_Init (Hubsensor_t* HS_data){
    if (HAL_UART_Receive_DMA(&huart3, (uint8_t *)UART3_RxBuff,14) != HAL_OK)
     {
 	   Error_Handler();
     }
}

void Hubsensor_Service (Hubsensor_t* HS_data){
	uint8_t i;
	i=13;
	while (i>0&&(UART3_RxBuff[i]>>4)!=0x00 && UART3_RxBuff[i-1]!=0xFF){
		i--;
	}

	//printf_("%d , %d, %d  %d\n ", i, UART3_RxBuff[i-7],(UART3_RxBuff[i-7]+UART3_RxBuff[i-6]+UART3_RxBuff[i-5]+UART3_RxBuff[i-4]+UART3_RxBuff[i-3])%256,UART3_RxBuff[i-2]+1);
	if ((UART3_RxBuff[i-7]+UART3_RxBuff[i-6]+UART3_RxBuff[i-5]+UART3_RxBuff[i-4]+UART3_RxBuff[i-3])%256 == UART3_RxBuff[i-2]+1){
		HS_data->HS_UARTFail = 0;
		HS_data->HS_Pedalposition = UART3_RxBuff[i-5]&127;
		HS_data->HS_Pedals_turning = UART3_RxBuff[i-5]>>7;
		torque_cumulated-=torque_cumulated>>4;

		if(torque_offset<(UART3_RxBuff[i-7]<<8)+UART3_RxBuff[i-6]){
			torque_cumulated+=((UART3_RxBuff[i-7]<<8)+UART3_RxBuff[i-6])-torque_offset;
			}
		else{
			if(torque_cumulated>0)torque_cumulated--;
			}
		HS_data->HS_Torque = torque_cumulated>>4;
		HS_data->HS_Wheel_turning = UART3_RxBuff[i-4]>>7;
		HS_data->HS_Wheeltime = ((UART3_RxBuff[i-4]&127)<<8)+UART3_RxBuff[i-3];
		//printf_("%d, %d, %d, %d, %d, %d, %d\r\n",i, HS_data->HS_Overtemperature, HS_data->HS_Pedalposition, HS_data->HS_Pedals_turning, HS_data->HS_Torque, HS_data->HS_Wheel_turning, HS_data->HS_Wheeltime );
		//printf_("%d\n",HS_data->HS_Torque);
		//printf_("%d\n",((UART3_RxBuff[i-7]<<8)+UART3_RxBuff[i-6]));
	}
	else {
		//printf_("F\n");
		HS_data->HS_UARTFail = 1;
	}
}
