/*
 * hubsensor.h
 *
 *  Created on: 18.09.2022
 *      Author: gaswerke
 */

#ifndef HUBSENSOR_H_
#define HUBSENSOR_H_

#include "main.h"

typedef struct
{
    uint8_t         HS_Pedals_turning;
    uint8_t         HS_Wheel_turning;
    uint16_t        HS_Torque;
    uint16_t        HS_Wheeltime;
    uint8_t         HS_UARTFail;
    uint8_t         HS_Pedalposition;

}Hubsensor_t;

void Hubsensor_Service(Hubsensor_t* HS_data);
void Hubsensor_Init (Hubsensor_t* HS_data);

#endif /* HUBSENSOR_H_ */
