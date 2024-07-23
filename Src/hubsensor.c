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

#define BYTES7
//#define BYTES8

uint8_t UART2_RxBuff[16];//Hub sends blocks of 8 bytes, one complete message must be within 16 bytes.
uint8_t HubMessage[8];
char buffer[64];
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

uint8_t torque_offset = 145;
uint16_t torque_max = 1024;
uint16_t torque_cumulated = 0;
uint16_t torque_recent;


#ifdef BYTES8

void Hubsensor_Init (Hubsensor_t* HS_data){
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)UART2_RxBuff,16) != HAL_OK)
     {
 	   Error_Handler();
     }
}

void Hubsensor_Service (Hubsensor_t* HS_data){
	// still don't know if there is any temperature information in the Byte stream :-(
	uint8_t i;
	uint8_t checksum=0;
	i=0;
	while (i<8&& UART2_RxBuff[i]!=0xFF){ //assuming 0xFF is the EndOfMessage Byte and the first byte has no information in the upper 3 bits.
		i++;
	}
	memcpy(HubMessage,UART2_RxBuff+i,8);
	//HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&HubMessage, 8);
	for (i = 0; i < 7; i++) {checksum+=HubMessage[i];}
	//printf_("%d\n",DMA1_Channel6->CNDTR);
	//printf_("%d , %d,\n ",checksum,HubMessage[7] );
	if (checksum==HubMessage[7]){
		HS_data->HS_UARTFail = 0;
		HS_data->HS_Temperature = HubMessage[1]-30; //Byte 2: 0~210 represents -30°C~180°C, for example: 30—0°C
		HS_data->HS_Pedalposition = HubMessage[3]&127;
		HS_data->HS_Pedals_turning = HubMessage[3]>>7;
		//filter torque value
		torque_recent =(((HubMessage[4]>>6)<<8)+HubMessage[2])-torque_offset;
		torque_cumulated-=torque_cumulated>>4;
		if(torque_recent>0&&torque_recent<torque_max-torque_offset){ //only cumulate torque, if value is in plausible range
			torque_cumulated+=torque_recent;
			}
		else{
			if(torque_cumulated>0)torque_cumulated--;
			}
		HS_data->HS_Torque = torque_cumulated>>4;
		HS_data->HS_Wheel_turning = (HubMessage[4]>>5)&1;
		if(((HubMessage[4]&31)<<8)+(HubMessage[5])<4501){ //safety reason, filter non plausible values
			HS_data->HS_Wheeltime = ((HubMessage[4]&31)<<8)+(HubMessage[5]);

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
#endif

#ifdef BYTES7

void Hubsensor_Init (Hubsensor_t* HS_data){
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)UART2_RxBuff,14) != HAL_OK)
     {
 	   Error_Handler();
     }
}

void Hubsensor_Service (Hubsensor_t* HS_data){
	// still don't know if there is any temperature information in the Byte stream :-(
	uint8_t i;
	uint8_t checksum=0;
	i=0;
	while (i<7&& UART2_RxBuff[i]!=0xFF){ //assuming 0xFF is the EndOfMessage Byte and the first byte has no information in the upper 3 bits.
		i++;
	}
	memcpy(HubMessage,UART2_RxBuff+i,7);
	//HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&HubMessage, 8);
	for (i = 0; i < 6; i++) {checksum+=HubMessage[i];}
	//printf_("%d\n",DMA1_Channel6->CNDTR);
	//printf_("%d , %d,\n ",checksum,HubMessage[7] );
	if (checksum==HubMessage[6]){
		HS_data->HS_UARTFail = 0;
		HS_data->HS_Pedalposition = HubMessage[3]&127;
		HS_data->HS_Pedals_turning = HubMessage[3]>>7;
		//filter torque value
		torque_cumulated-=torque_cumulated>>4;
		if(torque_offset<(HubMessage[1]<<8)+HubMessage[2]&&((HubMessage[1]<<8)+HubMessage[2])<torque_max){ //only cumulate torque, if value is in plausible range
			torque_cumulated+=((HubMessage[1]<<8)+HubMessage[2])-torque_offset;
			}
		else{
			if(torque_cumulated>0)torque_cumulated--;
			}
		HS_data->HS_Torque = torque_cumulated>>4;
		HS_data->HS_Wheel_turning = HubMessage[4]>>7;
		if(((HubMessage[4]&127)<<8)+HubMessage[5]<4501){ //safety reason, filter non plausible values
			HS_data->HS_Wheeltime = ((HubMessage[4]&127)<<8)+HubMessage[5];
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
#endif
