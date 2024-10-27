/*
Library for King-Meter displays

Copyright � 2015 Michael Fabry (Michael@Fabry.de)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


// Includes

#include "config.h"
#include "main.h"
#include "display_No_2.h"
#include "stm32f1xx_hal.h"
#include "print.h"

#if (DISPLAY_TYPE_NO2)

UART_HandleTypeDef huart1;

void No2_Service(No2_t* No2_ctx);
int calculate_checksum(unsigned char* frame_buf, uint8_t length);

uint8_t  lowByte(uint16_t word);
uint8_t  highByte(uint16_t word);

uint8_t pas_tolerance  = 0;
uint8_t wheel_magnets = 1;
uint8_t vcutoff = 30;
uint8_t spd_max1 = 25;
uint8_t ui8_RxLength=1;


/* Public functions (Prototypes declared by display_kingmeter.h) */

/****************************************************************************************************
 * KingMeter_Init() - Initializes the display object
 *
 ****************************************************************************************************/

void No2_Init (No2_t* No2_ctx){
    //Start UART with DMA
    if (HAL_UART_Receive_DMA(&huart1, (uint8_t *)No2_ctx->RxBuff, 64) != HAL_OK)
     {
 	   Error_Handler();
     }
}








/****************************************************************************************************
 * KM_901U_Service() - Communication protocol of 901U firmware
 *
 ***************************************************************************************************/
void No2_Service(No2_t* No2_ctx)
{
	static uint8_t  TxBuffer[14] = {0x2,0x0E,0x1,0x0,0x80,0x0,0x0,0x2C,0x0,0xF9,0x0,0x0,0xFF,0xA};
    static uint8_t  last_pointer_position;
    static uint8_t  recent_pointer_position;
    static uint8_t  Rx_message_length;
    static uint8_t  No2_Message[32];

    recent_pointer_position = 64-DMA1_Channel5->CNDTR;

    if(recent_pointer_position>last_pointer_position){
    	Rx_message_length=recent_pointer_position-last_pointer_position;
    	//printf_("groesser %d, %d, %d \n ",recent_pointer_position,last_pointer_position, Rx_message_length);
    	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    	memcpy(No2_Message,No2_ctx->RxBuff+last_pointer_position,Rx_message_length);
    	//HAL_UART_Transmit(&huart3, (uint8_t *)&No2_Message, Rx_message_length,50);
	}
    else {
    	Rx_message_length=recent_pointer_position+64-last_pointer_position;
     	memcpy(No2_Message,No2_ctx->RxBuff+last_pointer_position,64-last_pointer_position);
        memcpy(No2_Message+64-last_pointer_position,No2_ctx->RxBuff,recent_pointer_position);
      //  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


    }
    last_pointer_position=recent_pointer_position;
    //HAL_UART_Transmit(&huart3, (uint8_t *)&No2_Message, Rx_message_length,50);

    if(No2_Message[19]==calculate_checksum(No2_Message, 20)){
    	//to do
    	No2_ctx->Rx.AssistLevel = No2_Message[4];
    	No2_ctx->Rx.NumberOfPasMagnets = No2_Message[18];
    	No2_ctx->Rx.CUR_Limit_A = No2_Message[13];
    	No2_ctx->Rx.Voltage_min_x10 = (No2_Message[14]<<8)+No2_Message[15];
    	No2_ctx->Rx.Throttle_mode = No2_Message[3];
    	TxBuffer[13]=calculate_checksum(TxBuffer, 14);
    	HAL_UART_Transmit(&huart1, (uint8_t *)&TxBuffer,14,50);
    }









    }



#endif

uint8_t lowByte(uint16_t word){
	return word & 0xFF;
}

uint8_t  highByte(uint16_t word){
	return word >>8;
}

int calculate_checksum(unsigned char* frame_buf, uint8_t length) {
  unsigned char xor = 0;
  unsigned char *p;
  unsigned char tmp;
  for (p = frame_buf; p < frame_buf + (length-1); p++) {
    tmp = *p;
    //printf("%d, %x\r\n ", p,tmp);
    xor = xor ^ tmp;
  }
  return(xor);
}
