/*
 * display_kunteng.c
 *
 *  Created on: 25.09.2019
 *      Author: Admin
 */

#include "main.h"
#include "display_kunteng.h"
#include "stm32f1xx_hal.h"
#include "eeprom.h"

static uint8_t ui8_tx_buffer[12];
uint8_t ui8_j;
uint8_t ui8_crc;
uint8_t ui8_last_XOR;
uint16_t ui16_wheel_period_ms =4500;
uint32_t ui32_battery_volts= 36;
uint8_t ui8_battery_soc = 12;
uint8_t ui16_error;
static uint8_t ui8_rx_buffer[13];
uint8_t ui8_rx_initial_buffer[13];
uint8_t ui8_rx_buffer_counter = 0;
uint8_t ui8_byte_received;
uint8_t ui8_moving_indication = 0;
uint8_t ui8_UARTCounter = 0;
uint8_t ui8_msg_received=0;
int16_t i16_eeprom_temp=0;
uint8_t ui8_gear_ratio = GEAR_RATIO;

volatile struc_lcd_configuration_variables lcd_configuration_variables;

extern UART_HandleTypeDef huart1;
void check_recent(void);

void kunteng_init()
{

    if (HAL_UART_Receive_DMA(&huart1, (uint8_t *)ui8_rx_buffer, 13) != HAL_OK)
     {
 	   Error_Handler();
     }

    EE_ReadVariable(EEPROM_KT_B0_B3, &i16_eeprom_temp);
    ui8_rx_buffer[0] = i16_eeprom_temp>>8;
    ui8_rx_initial_buffer[0] = i16_eeprom_temp>>8;
    ui8_rx_buffer[3] = i16_eeprom_temp &0xFF;
    ui8_rx_initial_buffer[3] = i16_eeprom_temp &0xFF;

    EE_ReadVariable(EEPROM_KT_B2_B4, &i16_eeprom_temp);
    ui8_rx_buffer[2] = i16_eeprom_temp>>8;
    ui8_rx_initial_buffer[2] = i16_eeprom_temp>>8;
    ui8_rx_buffer[4] = i16_eeprom_temp &0xFF;
    ui8_rx_initial_buffer[4] = i16_eeprom_temp &0xFF;

    EE_ReadVariable(EEPROM_KT_B6_B7, &i16_eeprom_temp);
    ui8_rx_buffer[6] = i16_eeprom_temp>>8;
    ui8_rx_initial_buffer[6] = i16_eeprom_temp>>8;
    ui8_rx_buffer[7] = i16_eeprom_temp &0xFF;
    ui8_rx_initial_buffer[7] = i16_eeprom_temp &0xFF;

    EE_ReadVariable(EEPROM_KT_B8_B9, &i16_eeprom_temp);
    ui8_rx_buffer[8] = i16_eeprom_temp>>8;
    ui8_rx_initial_buffer[8] = i16_eeprom_temp>>8;
    ui8_rx_buffer[9] = i16_eeprom_temp &0xFF;
    ui8_rx_initial_buffer[9] = i16_eeprom_temp &0xFF;

    EE_ReadVariable(EEPROM_KT_B1_B10, &i16_eeprom_temp);
    ui8_rx_buffer[1] = i16_eeprom_temp>>8;
    ui8_rx_initial_buffer[1] = i16_eeprom_temp>>8;
    ui8_rx_buffer[10] = i16_eeprom_temp &0xFF;
    ui8_rx_initial_buffer[10] = i16_eeprom_temp &0xFF;

    ui8_crc = 0;

    for (ui8_j = 0; ui8_j <= 12; ui8_j++)
    {
      if (ui8_j == 5) continue; // don't xor B5 (B7 in our case)
      ui8_crc ^= ui8_rx_buffer[ui8_j];
    }
    ui8_crc ^=10; //right XOR must be pasted here!!!
    ui8_rx_buffer [5]=ui8_crc;
    ui8_rx_buffer[11]=0x37;
    ui8_rx_buffer[12]=0x0E;

}

void display_update(MotorState_t* MS_U)
{

   // prepare moving indication info
  ui8_moving_indication = 0;
if (!HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)) { ui8_moving_indication |= (1 << 5); }
  //if (ebike_app_cruise_control_is_set ()) { ui8_moving_indication |= (1 << 3); }
  if (throttle_is_set ()) { ui8_moving_indication |= (1 << 1); }
  //if (pas_is_set ()) { ui8_moving_indication |= (1 << 4); }


  // calc battery pack state of charge (SOC)
  ui32_battery_volts =  (MS_U->Voltage*CAL_BAT_V*256)/10000;  //hier noch die richtige Kalibrierung einbauen (*256 für bessere Auflösung)
  if (ui32_battery_volts > ((uint16_t) BATTERY_PACK_VOLTS_80)) { ui8_battery_soc = 16; } // 4 bars | full
  else if (ui32_battery_volts > ((uint16_t) BATTERY_PACK_VOLTS_60)) { ui8_battery_soc = 12; } // 3 bars
  else if (ui32_battery_volts > ((uint16_t) BATTERY_PACK_VOLTS_40)) { ui8_battery_soc = 8; } // 2 bars
  else if (ui32_battery_volts > ((uint16_t) BATTERY_PACK_VOLTS_20)) { ui8_battery_soc = 4; } // 1 bar
  else { ui8_battery_soc = 3; } // empty
  //ui16_wheel_period_ms = (MS_U->Speed*PULSES_PER_REVOLUTION)>>3; //for External speedsensor
  ui16_wheel_period_ms= ((MS_U->Speed)*6*(ui8_gear_ratio/2)/500);
  //ui16_wheel_period_ms= ((MS_U->Speed)*6*GEAR_RATIO/500); //*6 because 6 hall interrupts per revolution, /500 because of 500 kHz timer setting
  ui8_tx_buffer [0] =  65;
  // B1: battery level
  ui8_tx_buffer [1] = ui8_battery_soc;
  // B2: 24V controller
  ui8_tx_buffer [2] = (uint8_t) COMMUNICATIONS_BATTERY_VOLTAGE;
  // B3: speed, wheel rotation period, ms; period(ms)=B3*256+B4;
  ui8_tx_buffer [3] = (ui16_wheel_period_ms >> 8) & 0xff;
  ui8_tx_buffer [4] = (ui16_wheel_period_ms) & 0xff;



  // B5: error info display
  ui8_tx_buffer [5] = ui16_error;
  // B6: CRC: xor B1,B2,B3,B4,B5,B7,B8,B9,B10,B11
  // 0 value so no effect on xor operation for now
  ui8_tx_buffer [6] = 0;
  // B7: moving mode indication, bit
  // throttle: 2
  ui8_tx_buffer [7] = ui8_moving_indication;
  // B8: 4x controller current
  // Vbat = 30V:
  // - B8 = 255, LCD shows 1912 watts
  // - B8 = 250, LCD shows 1875 watts
  // - B8 = 100, LCD shows 750 watts
  // each unit of B8 = 0.25A


  //ui8_tx_buffer [8] =  (uint8_t)(((ui16_BatteryCurrent-ui16_current_cal_b+1)<<2)/current_cal_a);
  ui8_tx_buffer [8] =  (uint8_t)(MS_U->Battery_Current*MS_U->Voltage*CAL_BAT_V/82010000);   //Kalibrierung nach Binatone, empririsch ermittelt. Strom und Spannung in Milli, 13W pro digit
  // B9: motor temperature
  ui8_tx_buffer [9] = MS_U->Temperature-15; //according to documentation at endless sphere	
  // B10 and B11: 0
  ui8_tx_buffer [10] = 0;
  ui8_tx_buffer [11] = 0;

  // calculate CRC xor
  ui8_crc = 0;
  for (ui8_j = 1; ui8_j <= 11; ui8_j++)
  {
    ui8_crc ^= ui8_tx_buffer[ui8_j];
  }
  ui8_tx_buffer [6] = ui8_crc;

  // send the package over UART
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&ui8_tx_buffer, 12);
}

/********************************************************************************************/
 // Process received package from the LCD
 //

 // see if we have a received package to be processed
void check_message(MotorState_t* MS_D, MotorParams_t* MP_D)
 {
  //printf("Byte recieved \r\n");
  // validation of the package data
   ui8_crc = 0;

   for (ui8_j = 0; ui8_j <= 12; ui8_j++)
   {
     if (ui8_j == 5) continue; // don't xor B5 (B7 in our case)
     ui8_crc ^= ui8_rx_buffer[ui8_j];
   }

   // check if end of message is OK
   if((ui8_rx_buffer[11]==0x32||ui8_rx_buffer[11]==0x37) && ui8_rx_buffer[12]==0x0E ){
	   // check if CRC is ok
   if (((ui8_crc ^ 10) == ui8_rx_buffer [5]) 	|| // some versions of CRC LCD5 (??)
	((ui8_crc ^ ui8_last_XOR) == ui8_rx_buffer [5])
	)
   { //printf("message valid \r\n");
     lcd_configuration_variables.ui8_assist_level = ui8_rx_buffer [1] & 7;
     lcd_configuration_variables.ui8_light = ui8_rx_buffer [1]>>7 & 1;
     lcd_configuration_variables.ui8_motor_characteristic = ui8_rx_buffer [3];
     lcd_configuration_variables.ui8_wheel_size = ((ui8_rx_buffer [4] & 192) >> 6) | ((ui8_rx_buffer [2] & 7) << 2);
     lcd_configuration_variables.ui8_max_speed = (10 + ((ui8_rx_buffer [2] & 248) >> 3)) | (ui8_rx_buffer [4] & 32);
     lcd_configuration_variables.ui8_power_assist_control_mode = ui8_rx_buffer [4] & 8;
     lcd_configuration_variables.ui8_controller_max_current = (ui8_rx_buffer [7] & 15);
     MS_D->assist_level = lcd_configuration_variables.ui8_assist_level;
     MP_D->speedLimit = lcd_configuration_variables.ui8_max_speed;

		lcd_configuration_variables.ui8_p1 = ui8_rx_buffer[3];
		lcd_configuration_variables.ui8_p2 = ui8_rx_buffer[4] & 0x07;
		lcd_configuration_variables.ui8_p3 = ui8_rx_buffer[4] & 0x08;
		lcd_configuration_variables.ui8_p4 = ui8_rx_buffer[4] & 0x10;
		lcd_configuration_variables.ui8_p5 = ui8_rx_buffer[0];

		lcd_configuration_variables.ui8_c1 = (ui8_rx_buffer[6] & 0x38) >> 3;
		lcd_configuration_variables.ui8_c2 = (ui8_rx_buffer[6] & 0x37);
		lcd_configuration_variables.ui8_c4 = (ui8_rx_buffer[8] & 0xE0) >> 5;
		lcd_configuration_variables.ui8_c5 = (ui8_rx_buffer[7] & 0x0F);
		lcd_configuration_variables.ui8_c12 = (ui8_rx_buffer[9] & 0x0F);
		lcd_configuration_variables.ui8_c13 = (ui8_rx_buffer[10] & 0x1C) >> 2;
		lcd_configuration_variables.ui8_c14 = (ui8_rx_buffer[7] & 0x60) >> 5;
		if(lcd_configuration_variables.ui8_p1 != ui8_gear_ratio){
				    	 ui8_gear_ratio=lcd_configuration_variables.ui8_p1;
				     }

     if(lcd_configuration_variables.ui8_light){
    	 HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);
    	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
     }
     else{
    	 HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
    	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
     }

     display_update(MS_D);
     check_recent(); //byte 1 contains the PAS level, that may be changed quite often. Better run only at system shutdown, due to limited possible write cycles to flash
   }//end CRC OK
   else{ //search for right last XOR

	   ui8_crc = 0;

	   for (ui8_j = 0; ui8_j <= 12; ui8_j++)
	   {
	     if (ui8_j == 5) continue; // don't xor B5 (B7 in our case)
	     ui8_crc ^= ui8_rx_buffer[ui8_j];
	   }
	   for (ui8_j = 0; ui8_j <= 50; ui8_j++)
	   {
	     if((ui8_crc ^ ui8_j) == ui8_rx_buffer [5]) ui8_last_XOR = ui8_j;
	   }
   }
   }// end EOT OK
   else{
	  //resyncronize the communication
	       CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
		   DMA1_Channel5->CNDTR=2;
		   SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);

		   if((ui8_rx_buffer[0]==0x32||ui8_rx_buffer[0]==0x37) && ui8_rx_buffer[1]==0x0E ){
	  	   CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
	  	   DMA1_Channel5->CNDTR=13;
	  	   SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
		   }

   }
 }

//check if differences between initial values and recent values, store to emulated EEPROM if necessary
/*
EEPROM_KT_B0_B3
EEPROM_KT_B2_B4
EEPROM_KT_B6_B7
EEPROM_KT_B8_B9
EEPROM_KT_B1_B10
*/
//byte 1 contains the PAS level, that may be changed quite often. Better run only at system shutdown.
void check_recent(void){

	if(ui8_rx_buffer[0]!=ui8_rx_initial_buffer[0] || ui8_rx_buffer[3]!=ui8_rx_initial_buffer[3] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B0_B3,ui8_rx_buffer[0]<<8 | ui8_rx_buffer[3] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[0]=ui8_rx_buffer[0];
	    ui8_rx_initial_buffer[3]=ui8_rx_buffer[3];
	}

	if(ui8_rx_buffer[2]!=ui8_rx_initial_buffer[2] || ui8_rx_buffer[4]!=ui8_rx_initial_buffer[4] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B2_B4,ui8_rx_buffer[2]<<8 | ui8_rx_buffer[4] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[2]=ui8_rx_buffer[2];
	    ui8_rx_initial_buffer[4]=ui8_rx_buffer[4];
	}

	if(ui8_rx_buffer[6]!=ui8_rx_initial_buffer[6] || ui8_rx_buffer[7]!=ui8_rx_initial_buffer[7] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B6_B7,ui8_rx_buffer[6]<<8 | ui8_rx_buffer[7] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[6]=ui8_rx_buffer[6];
	    ui8_rx_initial_buffer[7]=ui8_rx_buffer[7];
	}

	if(ui8_rx_buffer[8]!=ui8_rx_initial_buffer[8] || ui8_rx_buffer[9]!=ui8_rx_initial_buffer[9] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B8_B9,ui8_rx_buffer[8]<<8 | ui8_rx_buffer[9] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[8]=ui8_rx_buffer[8];
	    ui8_rx_initial_buffer[9]=ui8_rx_buffer[9];
	}
//only check Byte 10
	if( ui8_rx_buffer[10]!=ui8_rx_initial_buffer[10] ){
	    HAL_FLASH_Unlock();
	    EE_WriteVariable(EEPROM_KT_B1_B10,ui8_rx_buffer[1]<<8 | ui8_rx_buffer[10] );
	    HAL_FLASH_Lock();
	    ui8_rx_initial_buffer[1]=ui8_rx_buffer[1];
	    ui8_rx_initial_buffer[10]=ui8_rx_buffer[10];
	}


}
