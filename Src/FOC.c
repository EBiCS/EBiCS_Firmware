/*
 * FOC.c
 *
 *  Created on: 25.01.2019
 *      Author: Stancecoke
 */
#include "main.h"
#include "math.h"
#include "stm32f1xx_hal.h"
#include <arm_math.h>

void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, float flt_teta, int16_t int16_i_q_target);

float32_t PI_control_i_q (float ist, float soll);
float32_t PI_control_i_d (float ist, float soll);



void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, float flt_teta, int16_t int16_i_q_target)
{

	float32_t flt_i_alpha;
	float32_t flt_i_beta;
	float32_t flt_u_alpha;
	float32_t flt_u_beta;
	float32_t flt_i_d;
	float32_t flt_i_q;
	float32_t flt_u_d;
	float32_t flt_u_q;
	float32_t	sinevalue=0, cosinevalue=0;


	// Clark transformation
	arm_clarke_f32(int16_i_as, int16_i_bs, &flt_i_alpha, &flt_i_beta);
	arm_sin_cos_f32(flt_teta, &sinevalue, &cosinevalue);

	// Park transformation
	arm_park_f32(flt_i_alpha, flt_i_beta, &flt_i_d, &flt_i_q, sinevalue, cosinevalue);

	//Control iq
	flt_u_q =  PI_control_i_q(flt_i_q, int16_i_q_target);

	//Control id
	flt_u_d =  PI_control_i_q(flt_i_d, 0); //control direct current to zero

	//inverse Park transformation
	arm_inv_park_f32(flt_u_d, flt_u_q, &flt_u_alpha, &flt_u_beta, sinevalue, cosinevalue);

}
//PI Control for quadrature current iq (torque)
float32_t PI_control_i_q (float ist, float soll)
{
  float float_p;
  static float float_q_i;
  static float float_q_dc=0;
  float_p=((float)soll - (float)ist)*P_FACTOR_I_Q;
  float_q_i+=((float)soll - (float)ist)*I_FACTOR_I_Q;
    //avoid too big steps in one loop run
  if (float_p+float_q_i>float_q_dc+5) float_q_dc+=5;
  else if  (float_p+float_q_i<float_q_dc-5)float_q_dc-=5;
  else float_q_dc=float_p+float_q_i;

  return (float_q_dc);
}

float32_t PI_control_i_d (float ist, float soll)
  {
    float float_p;
    static float float_d_i;
    static float float_d_dc=0;
    float_p=((float)soll - (float)ist)*P_FACTOR_I_D;
    float_d_i+=((float)soll - (float)ist)*I_FACTOR_I_D;
    if (float_d_i<0)float_d_i=0;
    //avoid too big steps in one loop run
    if (float_p+float_d_i>float_d_dc+5) float_d_dc+=5;
    else if  (float_p+float_d_i<float_d_dc-5) float_d_dc-=5;
    else float_d_dc=float_p+float_d_i;

    return (float_d_dc);
  }

