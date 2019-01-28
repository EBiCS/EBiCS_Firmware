/*
 * FOC.c
 *
 *  Created on: 25.01.2019
 *      Author: Stancecoke
 */
#include "main.h"
#include "FOC.h"
#include "stm32f1xx_hal.h"
#include <arm_math.h>

//float32_t	T_halfsample = 0.00003125;
//float32_t	counterfrequency = 64000000;
//float32_t	U_max = (1/_SQRT3)*_U_DC;
//extern uint16_t	switchtime[3];



void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, float flt_teta, int16_t int16_i_q_target);
void svpwm(float32_t flt_u_alpha, float32_t flt_u_beta, float32_t flt_teta);
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


	//limit voltage in rotating frame, refer chapter 4.10.1 of UM1052
	float32_t	flt_u_abs = hypotf(flt_u_q,flt_u_d); //absolute value of U in static frame

	if (flt_u_abs > _U_MAX){
		flt_u_q *=_U_MAX/flt_u_abs;
		flt_u_d *=_U_MAX/flt_u_abs;
	}

	//inverse Park transformation
	arm_inv_park_f32(flt_u_d, flt_u_q, &flt_u_alpha, &flt_u_beta, sinevalue, cosinevalue);

	//call SVPWM calculation
	svpwm(flt_u_alpha, flt_u_beta, flt_teta*DEG_IN_RAD);

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

void svpwm(float32_t flt_u_alpha, float32_t flt_u_beta, float32_t flt_teta)	{ //you have to send angle teta in rad

//SVPWM according to chapter 4.9 of UM1052


	float32_t flt_U_alpha = (_SQRT3) *_T * flt_u_alpha;
	float32_t flt_U_beta = -_T * flt_u_beta;
	float32_t X = flt_U_beta;
	float32_t Y = (flt_U_alpha+flt_U_beta)/2;
	float32_t Z = (flt_U_beta-flt_U_alpha)/2;

	//Sector 1 & 4
	if ((Y>=0 && Z<0 && X>0)||(Y < 0 && Z>=0 && X<=0)){
		switchtime[0] = _T/4 + (_T/2+X-Z)/2;
		switchtime[1] = switchtime[0] + Z;
		switchtime[2] = switchtime[1] - X;
	}

	//Sector 2 & 5
	if ((Y>=0 && Z>=0) || (Y<0 && Z<0) ){
		switchtime[0] = _T/4 + (_T/2+Y-Z)/2;
		switchtime[1] = switchtime[0] + Z;
		switchtime[2] = switchtime[0] - Y;
	}

	//Sector 3 & 6
	if ((Y<0 && Z>=0 && X>0)||(Y >= 0 && Z<0 && X<=0)){
		switchtime[0] = _T/4 + (_T/2+Y-X)/2;
		switchtime[2] = switchtime[0] - Y;
		switchtime[1] = switchtime[2] + X;
	}

	// scale and shift to 16 bit timer range
	switchtime[0] = switchtime[0] * _f + 32768;
	switchtime[1] = switchtime[1] * _f + 32768;
	switchtime[2] = switchtime[2] * _f + 32768;


/*
    uint8_t	sector = flt_teta/_PIdiv3; //angle in rad, gives sector 1 .. 6
	float32_t	U_ref = hypotf(flt_u_alpha,flt_u_beta); //vector length
	if (U_ref > U_max) { //limit Uref to max value
		U_ref = U_max;
	}
	float32_t	angle = flt_teta - (sector*_PIdiv3);  //angle in rad gives lokal angel from 0..60°
	float32_t	U_ref_percent = (_SQRT3)*(U_ref/_U_DC); // previous: (2/_SQRT3) scales back
	float32_t	t_1 = U_ref_percent*arm_sin_f32(_PIdiv3-angle)*T_halfsample;
	float32_t	t_2 = U_ref_percent*arm_sin_f32(angle)*T_halfsample;
	float32_t	t_0 = T_halfsample - t_1 - t_2;
	float32_t	t_0_half = t_0/2;


	// Switching counter values for Timer Interrupts

	// Upper switches
	uint16_t	ontime_t_0_half = (t_0_half) * counterfrequency;
	uint16_t	ontime_value_1 = (t_0_half + t_1) * counterfrequency;
	uint16_t	ontime_value_2 = (t_0_half + t_2) * counterfrequency;
	uint16_t	ontime_value_3 = (t_0_half + t_1 + t_2) * counterfrequency;

	switch (sector)	{

		//				Upper switches

		// Sector 1
		case 0:		switchtime[0] = &ontime_t_0_half;
					switchtime[1] = &ontime_value_1;
					switchtime[2] = &ontime_value_3;
				break;

		// Sector 2
		case 1:		switchtime[0] = &ontime_value_2;
					switchtime[1] = &ontime_t_0_half;
					switchtime[2] = &ontime_value_3;
				break;

		// Sector 3
		case 2:		switchtime[0] = &ontime_value_3;
					switchtime[1] = &ontime_t_0_half;
					switchtime[2] = &ontime_value_1;
				break;

		// Sector 4
		case 3:		switchtime[0] = &ontime_value_3;
					switchtime[1] = &ontime_value_2;
					switchtime[2] = &ontime_t_0_half;
				break;

		// Sector 5
		case 4:		switchtime[0] = &ontime_value_1;
					switchtime[1] = &ontime_value_3;
					switchtime[2] = &ontime_t_0_half;
				break;

		// Sector 6
		case 5:		switchtime[0] = &ontime_t_0_half;
					switchtime[1] = &ontime_value_3;
					switchtime[2] = &ontime_value_2;
				break;
	} // end of switch */
}

