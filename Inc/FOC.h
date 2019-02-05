/*
 * FOC.h
 *
 *  Created on: 25.01.2019
 *      Author: stancecoke
 */

#ifndef FOC_H_
#define FOC_H_

#include <arm_math.h>

void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target);

// Maximum Voltage applying, a little less than 2^15
#define _U_MAX	2048
// Square Root of 3
#define _SQRT3	1.73205081
// Pi divided by 3
#define _PIdiv3	1.04719755
// PWM period




//globals



#endif /* FOC_H_ */
