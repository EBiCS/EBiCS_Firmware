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
#include "display_kingmeter.h"
#include "stm32f1xx_hal.h"
#include "print.h"

#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)

// Definitions
#define RXSTATE_STARTCODE   0
#define RXSTATE_SENDTXMSG   1
#define RXSTATE_MSGBODY     2
#define RXSTATE_DONE        3
uint8_t FirstRunFlag = 0;
extern UART_HandleTypeDef huart1;


#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
 const uint16_t KM_WHEELSIZE[8] = { KM_WHEELSIZE_16, KM_WHEELSIZE_18, KM_WHEELSIZE_20,  KM_WHEELSIZE_22,
                                    KM_WHEELSIZE_24, KM_WHEELSIZE_26, KM_WHEELSIZE_700, KM_WHEELSIZE_28 };
#endif



// Hashtable used for handshaking in 901U protocol
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
 const uint8_t KM_901U_HANDSHAKE[64] =
 {
 		137,
 		159,
 		134,
 		249,
 		88,
 		11,
 		250,
 		61,
 		33,
 		150,
 		3,
 		193,
 		118,
 		141,
 		209,
 		94,
 		226,
 		68,
 		146,
 		158,
 		145,
 		127,
 		216,
 		62,
 		116,
 		230,
 		101,
 		211,
 		251,
 		54,
 		229,
 		247,
 		20,
 		222,
 		59,
 		63,
 		35,
 		252,
 		142,
 		238,
 		23,
 		197,
 		84,
 		77,
 		147,
 		173,
 		210,
 		57,
 		142,
 		223,
 		157,
 		97,
 		36,
 		160,
 		229,
 		237,
 		75,
 		80,
 		37,
 		113,
 		154,
 		88,
 		23,
 		120
 };
#endif


// Local function prototypes
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
static void KM_618U_Service(KINGMETER_t* KM_ctx);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
static void KM_901U_Service(KINGMETER_t* KM_ctx);
#endif

static uint8_t  lowByte(uint16_t word);
static uint8_t  highByte(uint16_t word);

uint8_t pas_tolerance  = 0;
uint8_t wheel_magnets = 1;
uint8_t vcutoff = 30;
//uint16_t wheel_circumference = 2200;
uint8_t spd_max1 = 25;
uint8_t ui8_RxLength=1;


/* Public functions (Prototypes declared by display_kingmeter.h) */

/****************************************************************************************************
 * KingMeter_Init() - Initializes the display object
 *
 ****************************************************************************************************/

void KingMeter_Init (KINGMETER_t* KM_ctx)

{
    uint8_t i;


    KM_ctx->RxState                         = RXSTATE_STARTCODE;
    KM_ctx->DirectSetpoint                  = 0xFF; //hier aktuellen Timerwert als Startzeitpunkt

    for(i=0; i<KM_MAX_RXBUFF; i++)
    {
        KM_ctx->RxBuff[i]                   = 0x00;
    }

    KM_ctx->RxCnt                           = 0;

    // Settings received from display:
    KM_ctx->Settings.PAS_RUN_Direction      = KM_PASDIR_FORWARD;
    KM_ctx->Settings.PAS_SCN_Tolerance      = (uint8_t) pas_tolerance;
    KM_ctx->Settings.PAS_N_Ratio            = 255;
    KM_ctx->Settings.HND_HL_ThrParam        = KM_HND_HL_NO;
    KM_ctx->Settings.HND_HF_ThrParam        = KM_HND_HF_NO;
    KM_ctx->Settings.SYS_SSP_SlowStart      = 1;
    KM_ctx->Settings.SPS_SpdMagnets         = (uint8_t) wheel_magnets;
    KM_ctx->Settings.VOL_1_UnderVolt_x10    = (uint16_t) (vcutoff * 10);
    KM_ctx->Settings.WheelSize_mm           = (uint16_t) (WHEEL_CIRCUMFERENCE * 1000);

    // Parameters received from display in operation mode:

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
    KM_ctx->Rx.AssistLevel                  = 3; 					//J-LCD Level 1...5
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
	KM_ctx->Rx.AssistLevel                  = 128;					//MK5S Level 0...255
#endif

    KM_ctx->Rx.Headlight                    = KM_HEADLIGHT_OFF;
    KM_ctx->Rx.Battery                      = KM_BATTERY_NORMAL;
    KM_ctx->Rx.PushAssist                   = KM_PUSHASSIST_OFF;
    KM_ctx->Rx.PowerAssist                  = KM_POWERASSIST_ON;
    KM_ctx->Rx.Throttle                     = KM_THROTTLE_ON;
    KM_ctx->Rx.CruiseControl                = KM_CRUISE_OFF;
    KM_ctx->Rx.OverSpeed                    = KM_OVERSPEED_NO;
    KM_ctx->Rx.SPEEDMAX_Limit           = (uint16_t) (spd_max1 * 10);
    KM_ctx->Rx.CUR_Limit_mA                = 150;

    // Parameters to be send to display in operation mode:
    KM_ctx->Tx.Battery                      = KM_BATTERY_NORMAL;
    KM_ctx->Tx.Wheeltime_ms                 = KM_MAX_WHEELTIME;
    KM_ctx->Tx.Error                        = KM_ERROR_NONE;
    KM_ctx->Tx.Current_x10                  = 0;


#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
    //Start UART with DMA
    if (HAL_UART_Receive_DMA(&huart1, (uint8_t *)KM_ctx->RxBuff, KM_MAX_RXBUFF) != HAL_OK)
     {
 	   Error_Handler();
     }
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
    //Start UART with DMA
    if (HAL_UART_Receive_DMA(&huart1, (uint8_t *)KM_ctx->RxBuff, 64) != HAL_OK)
     {
 	   Error_Handler();
     }
#endif
   // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&buffer, KM_MAX_RXBUFF);
}



/****************************************************************************************************
 * KingMeter_Service() - Communicates data from and to the display
 *
 ***************************************************************************************************/
void KingMeter_Service(KINGMETER_t* KM_ctx)
{
    #if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
    KM_618U_Service(KM_ctx);
    #endif

    #if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
    KM_901U_Service(KM_ctx);
    #endif
}




/* Local functions */

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
/****************************************************************************************************
 * KM_618U_Service() - Communication protocol of 618U firmware (J-LCD compatible)
 *
 ***************************************************************************************************/
static void KM_618U_Service(KINGMETER_t* KM_ctx)
{
    uint8_t  i;
    static uint8_t TxBuffer[KM_MAX_TXBUFF];
    KM_ctx->RxState = RXSTATE_SENDTXMSG;

// Send message to display


    if(KM_ctx->RxState == RXSTATE_SENDTXMSG)
    {
        KM_ctx->RxState = RXSTATE_MSGBODY;

        // Prepare Tx message
        TxBuffer[0] = 0X46;                                               // StartCode

        if(KM_ctx->Tx.Battery == KM_BATTERY_LOW)
        {
            TxBuffer[1] = 0x00;                                           // If none of Bit[0..2] is set, display blinks
        }
        else
        {
            TxBuffer[1] = 0x01;
        }

        TxBuffer[2] = (uint8_t) ((KM_ctx->Tx.Current_x10 * 3) / 10);      // Current unit: 1/3A
        TxBuffer[3] = highByte(KM_ctx->Tx.Wheeltime_ms);  //High byte of wheeltime
        TxBuffer[4] = lowByte(KM_ctx->Tx.Wheeltime_ms);  // Mask lower 8 bits
        TxBuffer[5] = 0x7A;                                               // Reply with WheelSize 26" / Maxspeed 25km/h (no influence on display)
        TxBuffer[6] = KM_ctx->Tx.Error;

        
        // Send prepared message
        TxBuffer[7] = 0x00;

       // KM_ctx->SerialPort->write(TxBuffer[0]);                           // Send StartCode

        for(i=1; i<7; i++)
        {
   //         KM_ctx->SerialPort->write(TxBuffer[i]);                       // Send TxBuffer[1..6]
            TxBuffer[7] = TxBuffer[7] ^ TxBuffer[i];                          // Calculate XOR CheckSum
        }

     //   KM_ctx->SerialPort->write(TxBuffer[7]);                           // Send XOR CheckSum



    }


    // Receive Message body
    if(KM_ctx->RxState == RXSTATE_MSGBODY)
    {

                // Verify XOR CheckSum
                if(KM_ctx->RxBuff[4] == (KM_ctx->RxBuff[1] ^ KM_ctx->RxBuff[2] ^ KM_ctx->RxBuff[3]))
                {
                    KM_ctx->RxState = RXSTATE_DONE;
                }
                else
                {
                    KM_ctx->RxState = RXSTATE_STARTCODE;
                }



    }

    // Message received completely
    if(KM_ctx->RxState == RXSTATE_DONE)
    {

        // Buffer ueber DMA senden
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&TxBuffer, KM_MAX_TXBUFF);
       // HAL_Delay(6);

        KM_ctx->RxState = RXSTATE_STARTCODE;

        // Decode PAS level - Display sets PAS-level to 0 when overspeed detected!
        KM_ctx->Rx.AssistLevel = KM_ctx->RxBuff[1] & 0x07;

        // Decode Headlight status
        KM_ctx->Rx.Headlight = (KM_ctx->RxBuff[1] & 0x80) >> 7;         // KM_HEADLIGHT_OFF / KM_HEADLIGHT_ON

//      KM_ctx->Rx.Battery;

        // Decode PushAssist status
        KM_ctx->Rx.PushAssist = (KM_ctx->RxBuff[1] & 0x10) >> 4;        // KM_PUSHASSIST_OFF / KM_PUSHASSIST_ON

//      KM_ctx->Rx.PowerAssist;
//      KM_ctx->Rx.Throttle;
//      KM_ctx->Rx.CruiseControl;
//      KM_ctx->Rx.OverSpeed;

        // Decode Speedlimit
        KM_ctx->Rx.SPEEDMAX_Limit= (((KM_ctx->RxBuff[2] & 0xF8) >> 3) + 10) * 10;

        // Decode Wheelsize by hashtable
        KM_ctx->Settings.WheelSize_mm = KM_WHEELSIZE[KM_ctx->RxBuff[2] & 0x07];

//      KM_ctx->Rx.CUR_Limit_x10;
    }
}
#endif




#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
/****************************************************************************************************
 * KM_901U_Service() - Communication protocol of 901U firmware
 *
 ***************************************************************************************************/
static void KM_901U_Service(KINGMETER_t* KM_ctx)
{
	static uint8_t  TxBuffer[KM_MAX_TXBUFF];
	static uint8_t  m;
    static uint8_t  last_pointer_position;
    static uint8_t  recent_pointer_position;


    uint16_t CheckSum;

    static uint8_t  TxCnt;
    static uint8_t  Rx_message_length;
    static uint8_t  KM_Message[32];

    recent_pointer_position = 64-DMA1_Channel5->CNDTR;

    if(recent_pointer_position>last_pointer_position){
    	Rx_message_length=recent_pointer_position-last_pointer_position;
    	//printf_("groesser %d, %d, %d \n ",recent_pointer_position,last_pointer_position, Rx_message_length);
    	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    	memcpy(KM_Message,KM_ctx->RxBuff+last_pointer_position,Rx_message_length);
    	//HAL_UART_Transmit(&huart3, (uint8_t *)&KM_Message, Rx_message_length,50);
	}
    else {
    	Rx_message_length=recent_pointer_position+64-last_pointer_position;
     	memcpy(KM_Message,KM_ctx->RxBuff+last_pointer_position,64-last_pointer_position);
        memcpy(KM_Message+64-last_pointer_position,KM_ctx->RxBuff,recent_pointer_position);
      //  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


    }
    last_pointer_position=recent_pointer_position;
    //HAL_UART_Transmit(&huart3, (uint8_t *)&KM_Message, Rx_message_length,50);




            	CheckSum = 0x0000;
            	for(m=1; m<(4+KM_Message[3]); m++)
            		{
            		CheckSum = CheckSum + KM_Message[m];            // Calculate CheckSum
            		}
           		CheckSum-=(KM_Message[m]+((KM_Message[m+1])<<8));

     			switch(KM_Message[2])
    			        {
    			            case 0x52:      // Operation mode
    			            	if(!CheckSum) //low-byte and high-byte
    			            		{

    			            		//HAL_UART_Transmit(&huart3, (uint8_t *)&KM_Message, Rx_message_length,50);
    			                // Decode Rx message

    			                KM_ctx->Rx.AssistLevel        =  KM_Message[4];                 // 0..255
    			                KM_ctx->Rx.Headlight          = (KM_Message[5] & 0xC0) >> 6;    // KM_HEADLIGHT_OFF / KM_HEADLIGHT_ON / KM_HEADLIGHT_LOW / KM_HEADLIGHT_HIGH
    			                KM_ctx->Rx.Battery            = (KM_Message[5] & 0x20) >> 5;    // KM_BATTERY_NORMAL / KM_BATTERY_LOW
    			                KM_ctx->Rx.PushAssist         = (KM_Message[5] & 0x10) >> 4;    // KM_PUSHASSIST_OFF / KM_PUSHASSIST_ON
    			                KM_ctx->Rx.PowerAssist        = (KM_Message[5] & 0x08) >> 3;    // KM_POWERASSIST_OFF / KM_POWERASSIST_ON
    			                KM_ctx->Rx.Throttle           = (KM_Message[5] & 0x04) >> 2;    // KM_THROTTLE_OFF / KM_THROTTLE_ON
    			                KM_ctx->Rx.CruiseControl      = (KM_Message[5] & 0x02) >> 1;    // KM_CRUISE_OFF / KM_CRUISE_ON
    			                KM_ctx->Rx.OverSpeed          = (KM_Message[5] & 0x01);         // KM_OVERSPEED_NO / KM_OVERSPEED_YES

   			            		}
    			            	else {// printf_("Checksum fail! \n ");



    			            	}

    			            	kingmeter_update();
    			                // Prepare Tx message
    			                TxBuffer[0]  = 0X3A;                                      // StartCode
    			                TxBuffer[1]  = 0x1A;                                      // SrcAdd:  Controller
    			                TxBuffer[2]  = 0x52;                                      // CmdCode
    			                TxBuffer[3]  = 0x05;                                      // DataSize


    			                if(KM_ctx->Tx.Battery == KM_BATTERY_LOW)
    			                {
    			                    TxBuffer[4]  = 0x40;                                  // State data (only UnderVoltage bit has influence on display)
    			                }
    			                else
    			                {														//Byte7 ist autocruise Symbol, Byte6 ist Battery low Symbol
    			                    TxBuffer[4]  = 0b00000000;                                  // State data (only UnderVoltage bit has influence on display)
    			                }

    			                TxBuffer[5]  = (uint8_t) ((KM_ctx->Tx.Current_x10 * 3) / 10);        			// Current low Strom in 1/3 Ampere, nur ein Byte
    			                TxBuffer[6]  = highByte(KM_ctx->Tx.Wheeltime_ms);         // WheelSpeed high Hinweis
    			                TxBuffer[7]  = lowByte (KM_ctx->Tx.Wheeltime_ms);         // WheelSpeed low
    			                TxBuffer[8] =  KM_ctx->Tx.Error;                          // Error

    			                TxCnt = 9;
    			                break;


    			            case 0x53:      // Settings mode

    			                // Decode Rx message
    			            	if(!CheckSum) //low-byte and high-byte
    			            		{

    			                KM_ctx->Settings.PAS_RUN_Direction   = (KM_Message[4] & 0x80) >> 7; // KM_PASDIR_FORWARD / KM_PASDIR_BACKWARD
    			                KM_ctx->Settings.ExecAutodetect   	 = (KM_Message[6]>>7)&0x01; 	// Execute Autodetect (with P18 of EN06 protocol)
    			                KM_ctx->Settings.Reverse   	 		 = (KM_Message[6]>>6)&0x01; 	// set spinning direction (with P19 of EN06 protocol)
    			                KM_ctx->Settings.PAS_SCN_Tolerance   =  KM_Message[5];              // 2..9
    			                KM_ctx->Settings.PAS_N_Ratio         =  KM_Message[6];              // 0..255
    			                KM_ctx->Settings.HND_HL_ThrParam     = (KM_Message[7] & 0x80) >> 7; // KM_HND_HL_NO / KM_HND_HL_YES
    			                KM_ctx->Settings.HND_HF_ThrParam     = (KM_Message[7] & 0x40) >> 6; // KM_HND_HF_NO / KM_HND_HF_YES
    			                KM_ctx->Settings.SYS_SSP_SlowStart   =  KM_Message[8];              // 1..9
    			                KM_ctx->Settings.SPS_SpdMagnets      =  KM_Message[9];             // 1..4
    			                KM_ctx->Settings.VOL_1_UnderVolt_x10 = (((uint16_t) KM_Message[11])<<8) | KM_Message[11];
    			                KM_ctx->Settings.WheelSize_mm        = (((uint16_t) KM_Message[12])<<8) | KM_Message[13];
    			    	        KM_ctx->Rx.SPEEDMAX_Limit          		= KM_Message[11];
    			    	        KM_ctx->Rx.CUR_Limit_mA                 = (KM_Message[8]&0x3F)*500;

    			    	        kingmeter_update();
    			    	        if(KM_ctx->Settings.ExecAutodetect&&!FirstRunFlag){
    			    	        	autodetect();
    			    	        	FirstRunFlag=1;
    			    	        }
    			    	        //if(KM_ctx->Rx.CUR_Limit_mA==21500)autodetect();
    			    	        //if(KM_ctx->Rx.CUR_Limit_mA==20500)get_internal_temp_offset();
    			            		}

    			                // Prepare Tx message with handshake code
    			                TxBuffer[0] = 0X3A;                                       // StartCode
    			                TxBuffer[1] = 0x1A;                                       // SrcAdd:  Controller
    			                TxBuffer[2] = 0x53;                                      	// CmdCode
    			                TxBuffer[3] = 0x05;                                       // Number of Databytes
    			                TxBuffer[4] = 0x00;
    			                TxBuffer[5] = 0x00;
    			                TxBuffer[6] = 0x0D;
    			                TxBuffer[7] = KM_901U_HANDSHAKE[KM_Message[9]];
    			                TxBuffer[8] = 0x00;
    			                TxBuffer[9] = 0x0C;
    			                TxBuffer[10] = 0x01;
    			                TxBuffer[11] = 0x0D;
    			                TxBuffer[12] = 0x0A;


    			               // 3A 1A 53 05 00 00 0D 91 00 10 01 0D 0A
    			                //3A 1A 53 05 80 00 0D 91 26 B6 01 0D 0A
    			                //3A 1A 53 05 00 00 0D 91 00 10 01 0D 0A
    			               // 3A 1A 53 05 00 00 0D 8D 00 0C 01 0D 0A
    			                														// DataSize
    			                //TxBuffer[5] = KM_901U_HANDSHAKE[KM_ctx->RxBuff[14]];      // Handshake answer
    			                TxCnt = 9;
    			                break;

    			            case 0x54:      // Operation mode
    			            	if(!CheckSum) //low-byte and high-byte
    			            		{
    			            		KM_ctx->DirectSetpoint=KM_Message[4];
    			            		}
    			            	break;

    			            default:
    			                TxCnt = 0;
    			        }


    			        // Send prepared message
    			        if(TxCnt && !CheckSum)
    			        {
    			            CheckSum = 0x0000;



    			            for(m=1; m<TxCnt; m++)
    			            {

    			                CheckSum = CheckSum + TxBuffer[m];                        // Calculate CheckSum
    			            }
    			            TxBuffer[TxCnt+0]=lowByte(CheckSum);							// Low Byte of checksum
    			            TxBuffer[TxCnt+1]=highByte(CheckSum);								// High Byte of checksum
    			            TxBuffer[TxCnt+2] = 0x0D;
    			            TxBuffer[TxCnt+3] = 0x0A;

    			            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&TxBuffer, TxCnt+4);
    			            //HAL_UART_Transmit(&huart3, (uint8_t *)&TxBuffer, TxCnt+4,50);
    			            //printf_("%d, %d \n ",TxCnt+4,KM_Message[2]);
    			        }





    }



#endif

uint8_t lowByte(uint16_t word){
	return word & 0xFF;
}

uint8_t  highByte(uint16_t word){
	return word >>8;
}


#endif // (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
