/*
Library for King-Meter displays

Copyright © 2015 Michael Fabry (Michael@Fabry.de)

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


#ifndef KINGMETER_H
#define KINGMETER_H


// Includes
#include "config.h"
#include "stdint.h"

#if (DISPLAY_TYPE & DISPLAY_TYPE_NO2)


typedef struct
{
    // Parameters received from display in operation mode:
    uint8_t  AssistLevel;               // Byte 4, Level in Display
    uint8_t  NumberOfPasMagnets;        // Byte 18, Bits 0-3
    uint8_t  Headlight;                 // Byte 5, Bit 5
    uint16_t WheelSizeInch_x10;         // HiByte 14, LowByte 15
    uint8_t  PushAssist;                // Byte 5, Bit 2
    uint8_t  ZeroStart;                 // Byte 5, Bit 6
    uint8_t  Throttle_mode;             // Byte 3, numbers not read out yet
    uint8_t  Start_delay_PAS;           // Byte 9, same Number as in display setting
    uint8_t  CruiseControl;             // Byte 18, Bit 6
    uint8_t  GearRatio;                 // Byte 6, same Number as in display setting
    uint8_t  BoostPower;                // Byte 10, same Number as in display setting
    uint16_t SPEEDMAX_Limit;        	// Byte 15, same Number as in display setting
    uint16_t Voltage_min_x10;           // HiByte 14, LowByte 15
    uint8_t  CUR_Limit_A;               // Byte 13, same Number as in display setting

}RX_PARAM_t;





typedef struct
{
    // Parameters to be send to display in operation mode:
    uint8_t  Battery;                   // KM_BATTERY_NORMAL / KM_BATTERY_LOW
    uint16_t Wheeltime_ms;              // Unit:1ms
    uint8_t  Error;                     // KM_ERROR_NONE, ..
    uint16_t Current_x10;               // Unit: 0.1A
    uint8_t BrakeActive;               // Unit: 0.1A

}TX_PARAM_t;


 #define KM_MAX_RXBUFF 64


typedef struct
{
    uint8_t         RxState;
    int8_t          DirectSetpoint;

    uint8_t         RxBuff[KM_MAX_RXBUFF];
    uint8_t         RxCnt;
    RX_PARAM_t      Rx;
    TX_PARAM_t      Tx;

}No2_t;




// Public function prototypes




void No2_Service(No2_t* No2_ctx);

void No2_Init(No2_t* No2_ctx);


#endif // Display Type Kingmeter

#endif // KINGMETER_H

