/*
 * FOC.h
 *
 *  Created on: 25.01.2019
 *      Author: stancecoke
 */

#ifndef FOC_H_
#define FOC_H_

#include <arm_math.h>
//exportetd functions
void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target, MotorState_t* MS_FOC);
void observer_update(long long v_alpha, long long v_beta, long long i_alpha, long long i_beta, q31_t *e_alpha, q31_t *e_beta);
q31_t atan2_LUT(q31_t e_alpha, q31_t e_beta);
q31_t PI_control_i_q (q31_t ist, q31_t soll);
q31_t PI_control_i_d (q31_t ist, q31_t soll);
q31_t PI_control_e_d (q31_t ist, q31_t soll);

// Maximum Voltage applyed
#define _U_MAX	1700L  //little lower than period of timer1 for proper phase current reading. Could be improved by dynamic timing of AD-conversion
// Square Root of 3
#define _SQRT3	1.73205081

#define ADC_DUR 250//minimal duration for proper ADC reading deadtime + noise subsiding + sample time


//globals
extern long long temp1;
extern long long temp2;
extern q31_t temp3;
extern q31_t temp4;
extern q31_t temp5;
extern q31_t temp6;
extern char PI_flag;
extern char Obs_flag;


//current control variables
extern q31_t q31_i_q_fil;
extern q31_t q31_i_d_fil;
extern q31_t q31_u_d;
extern q31_t q31_u_q;


//observer variables

extern q31_t q31_e_d_obs;
extern q31_t q31_ed_i; //integral part
extern q31_t e_log[300][6];

extern q31_t fl_e_alpha_obs;
extern q31_t fl_e_beta_obs;


#endif /* FOC_H_ */
