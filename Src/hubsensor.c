/*
 * hubsensor.c
 *
 *  Created on: 18.09.2022
 *      Author: gaswerke
 */


#include "hubsensor.h"

#include "stm32f1xx_hal.h"


uint8_t UART2_RxBuff[64]; //Hub sends blocks of 8 bytes, one complete message must be within 16 bytes.
UART_HandleTypeDef huart2;

uint8_t torque_offset = 145;
uint16_t torque_max = 1024;
uint16_t torque_cumulated = 0;
uint16_t torque_recent;

void Hubsensor_Init (Hubsensor_t* HS_data){
    if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)UART2_RxBuff,64) != HAL_OK)
     {
 	   Error_Handler();
     }
}

void Hubsensor_Service (Hubsensor_t* HS_data){
	// still don't know if there is any temperature information in the Byte stream :-(
	uint8_t i;
    static uint8_t  last_pointer_position;
    static uint8_t  recent_pointer_position;
    static uint8_t  Rx_message_length;
    static uint8_t  Hub_Message[8];
//	i=15;
//	while (i>8&& UART2_RxBuff[i-1]!=0xFF){ //assuming 0xFF is the EndOfMessage Byte and the first byte has no information in the upper 3 bits.
//		i--;
//	}

    recent_pointer_position = 64-DMA1_Channel5->CNDTR;

    if(recent_pointer_position>last_pointer_position){
    	Rx_message_length=recent_pointer_position-last_pointer_position;
    	//printf_("groesser %d, %d, %d \n ",recent_pointer_position,last_pointer_position, Rx_message_length);
    	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    	memcpy(Hub_Message,UART2_RxBuff+last_pointer_position,Rx_message_length);
    	//HAL_UART_Transmit(&huart3, (uint8_t *)&KM_Message, Rx_message_length,50);
	}
    else {
    	Rx_message_length=recent_pointer_position+64-last_pointer_position;
     	memcpy(Hub_Message,UART2_RxBuff+last_pointer_position,64-last_pointer_position);
        memcpy(Hub_Message+64-last_pointer_position,UART2_RxBuff,recent_pointer_position);
      //  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


    }
    last_pointer_position=recent_pointer_position;
    HAL_UART_Transmit(&huart2, (uint8_t *)&Hub_Message, Rx_message_length,50);


	if ((Hub_Message[0]+Hub_Message[1]+Hub_Message[2]+Hub_Message[3]+Hub_Message[4]+Hub_Message[5])%256 == Hub_Message[6]+1){

		HS_data->HS_UARTFail = 0;
		HS_data->HS_Temperature = Hub_Message[0]-30; //Byte 2: 0~210 represents -30℃~180℃, for example：30—0℃
		HS_data->HS_Pedalposition = Hub_Message[2]&127;
		HS_data->HS_Pedals_turning = Hub_Message[2]>>7;
		//filter torque value
		torque_recent =(((Hub_Message[3]>>6)<<8)+Hub_Message[1])-torque_offset;
		torque_cumulated-=torque_cumulated>>4;
		if(torque_recent>0&&torque_recent<torque_max-torque_offset){ //only cumulate torque, if value is in plausible range
			torque_cumulated+=torque_recent;
			}
		else{
			if(torque_cumulated>0)torque_cumulated--;
			}
		HS_data->HS_Torque = torque_cumulated>>4;
		//HS_data->HS_Torque = torque_recent;
		HS_data->HS_Wheel_turning = (Hub_Message[3]>>5)&1;
		if(((Hub_Message[3]&31)<<8)+(Hub_Message[4])<4501){ //safety reason, filter non plausible values
			HS_data->HS_Wheeltime = ((Hub_Message[3]&31)<<8)+(Hub_Message[4]);

		}

	}
	else {


	}
}
