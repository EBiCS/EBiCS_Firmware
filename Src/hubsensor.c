/*
 * hubsensor.c
 *
 *  Created on: 18.09.2022
 *      Author: gaswerke
 */


#include "hubsensor.h"
#include "print.h"
#include "stm32f1xx_hal.h"
#include "FOC.h"


uint8_t UART2_RxBuff[16];//Hub sends blocks of 8 bytes, one complete message must be within 16 bytes.
uint8_t HubMessage[8];
char buffer[64];
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

uint8_t torque_offset = 145;
uint16_t torque_max = 1024;
uint16_t torque_cumulated = 0;
uint16_t torque_recent;

void Hubsensor_Init (Hubsensor_t* HS_data){
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)UART2_RxBuff,16) != HAL_OK)
     {
 	   Error_Handler();
     }
}

void Hubsensor_Service (Hubsensor_t* HS_data){
	// still don't know if there is any temperature information in the Byte stream :-(
	uint8_t i;
	i=0;
	while (i<8&& UART2_RxBuff[i]!=0xFF){ //assuming 0xFF is the EndOfMessage Byte and the first byte has no information in the upper 3 bits.
		i++;
	}
	memcpy(HubMessage,UART2_RxBuff+i,8);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&HubMessage, 8);
	temp5=UART2_RxBuff[i-7];
	temp6=i;
	//printf_("%d\n",DMA1_Channel6->CNDTR);
	//printf_("%d , %d, %d  %d\n ", i, UART2_RxBuff[i-7],(UART2_RxBuff[i-7]+UART2_RxBuff[i-6]+UART2_RxBuff[i-5]+UART2_RxBuff[i-4]+UART2_RxBuff[i-3])%256,UART2_RxBuff[i-2]+1);
	if (i>7&&(UART2_RxBuff[i-7]+UART2_RxBuff[i-6]+UART2_RxBuff[i-5]+UART2_RxBuff[i-4]+UART2_RxBuff[i-3])%256 == UART2_RxBuff[i-2]+1){
		HS_data->HS_UARTFail = 0;
		HS_data->HS_Pedalposition = UART2_RxBuff[i-5]&127;
		HS_data->HS_Pedals_turning = UART2_RxBuff[i-5]>>7;
		//filter torque value
		torque_cumulated-=torque_cumulated>>4;
		if(torque_offset<(UART2_RxBuff[i-7]<<8)+UART2_RxBuff[i-6]&&((UART2_RxBuff[i-7]<<8)+UART2_RxBuff[i-6])<torque_max){ //only cumulate torque, if value is in plausible range
			torque_cumulated+=((UART2_RxBuff[i-7]<<8)+UART2_RxBuff[i-6])-torque_offset;
			}
		else{
			if(torque_cumulated>0)torque_cumulated--;
			}
		HS_data->HS_Torque = torque_cumulated>>4;
		HS_data->HS_Wheel_turning = UART2_RxBuff[i-4]>>7;
		if(((UART2_RxBuff[i-4]&127)<<8)+UART2_RxBuff[i-3]<4501){ //safety reason, filter non plausible values
			HS_data->HS_Wheeltime = ((UART2_RxBuff[i-4]&127)<<8)+UART2_RxBuff[i-3];
		}
		//printf_("%d, %d, %d, %d, %d, %d, %d\r\n",i, HS_data->HS_Overtemperature, HS_data->HS_Pedalposition, HS_data->HS_Pedals_turning, HS_data->HS_Torque, HS_data->HS_Wheel_turning, HS_data->HS_Wheeltime );
		//printf_("%d\n",DMA1_Channel6->CNDTR);
		//printf_("%d\n",((UART2_RxBuff[i-7]<<8)+UART2_RxBuff[i-6]));
	}
	else {
		//printf_("F\n");
//		HS_data->HS_UARTFail = 1;
//	       CLEAR_BIT(DMA1_Channel6->CCR, DMA_CCR_EN);
//		   DMA1_Channel6->CNDTR=14;
//		   SET_BIT(DMA1_Channel6->CCR, DMA_CCR_EN);

	}
}
