/*
 * hubsensor.c
 *
 *  Created on: 18.09.2022
 *      Author: gaswerke
 */


#include "hubsensor.h"

#include "stm32f1xx_hal.h"


uint8_t UART3_RxBuff[16]; //Hub sends blocks of 8 bytes, one complete message must be within 16 bytes.
UART_HandleTypeDef huart3;

uint8_t torque_offset = 145;
uint16_t torque_max = 1024;
uint16_t torque_cumulated = 0;
uint16_t torque_recent;

void Hubsensor_Init (Hubsensor_t* HS_data){
    if (HAL_UART_Receive_DMA(&huart3, (uint8_t *)UART3_RxBuff,16) != HAL_OK)
     {
 	   Error_Handler();
     }
}

void Hubsensor_Service (Hubsensor_t* HS_data){
	// still don't know if there is any temperature information in the Byte stream :-(
	uint8_t i;
	i=15;
	while (i>8&& UART3_RxBuff[i-1]!=0xFF){ //assuming 0xFF is the EndOfMessage Byte and the first byte has no information in the upper 3 bits.
		i--;
	}


	if (i>8&&(UART3_RxBuff[i-8]+UART3_RxBuff[i-7]+UART3_RxBuff[i-6]+UART3_RxBuff[i-5]+UART3_RxBuff[i-4]+UART3_RxBuff[i-3])%256 == UART3_RxBuff[i-2]+1){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HS_data->HS_UARTFail = 0;
		HS_data->HS_Temperature = UART3_RxBuff[i-8]-30; //Byte 2: 0~210 represents -30℃~180℃, for example：30—0℃
		HS_data->HS_Pedalposition = UART3_RxBuff[i-6]&127;
		HS_data->HS_Pedals_turning = UART3_RxBuff[i-6]>>7;
		//filter torque value
		torque_recent =(((UART3_RxBuff[i-5]>>6)<<8)+UART3_RxBuff[i-7])-torque_offset;
		torque_cumulated-=torque_cumulated>>4;
		if(torque_recent>0&&torque_recent<torque_max-torque_offset){ //only cumulate torque, if value is in plausible range
			torque_cumulated+=torque_recent;
			}
		else{
			if(torque_cumulated>0)torque_cumulated--;
			}
		HS_data->HS_Torque = torque_cumulated>>4;
		//HS_data->HS_Torque = torque_recent;
		HS_data->HS_Wheel_turning = (UART3_RxBuff[i-5]>>5)&1;
		if(((UART3_RxBuff[i-5]&31)<<8)+(UART3_RxBuff[i-4])<4501){ //safety reason, filter non plausible values
			HS_data->HS_Wheeltime = ((UART3_RxBuff[i-5]&31)<<8)+(UART3_RxBuff[i-4]);

		}

	}
	else {

		HS_data->HS_UARTFail = 1;
	       CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
		   DMA1_Channel3->CNDTR=16;
		   SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);

	}
}
