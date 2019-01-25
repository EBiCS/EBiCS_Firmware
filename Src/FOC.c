/*
 * FOC.c
 *
 *  Created on: 25.01.2019
 *      Author: Stancecoke
 */
#include "main.h"
#include "math.h"
#include "stm32f1xx_hal.h"

void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, float flt_teta, int16_t int16_i_target);



void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, float flt_teta, int16_t int16_i_target)
{

	int16_t int16_i_alpha;
	int16_t int16_i_beta;
	int16_t int16_i_d;
	int16_t int16_i_q;

	// Clark transformation
	int16_i_alpha =  int16_i_as;
	int16_i_beta = (int16_i_as+2*int16_i_bs)/sqrt(3);

	// Park transformation
	int16_i_d = int16_i_alpha * cos(flt_teta) + int16_i_beta * sin(flt_teta);
	int16_i_q = int16_i_beta * cos(flt_teta) - int16_i_alpha * sin(flt_teta);

}
