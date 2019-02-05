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

//q31_t	T_halfsample = 0.00003125;
//q31_t	counterfrequency = 64000000;
//q31_t	U_max = (1/_SQRT3)*_U_DC;
//extern uint16_t	switchtime[3];
const q31_t _T = 2000;



void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target);
void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta);
q31_t PI_control_i_q (q31_t ist, q31_t soll);
q31_t PI_control_i_d (q31_t ist, q31_t soll);






void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target)
{

	q31_t q31_i_alpha;
	q31_t q31_i_beta;
	q31_t q31_u_alpha;
	q31_t q31_u_beta;
	q31_t q31_i_d;
	q31_t q31_i_q;
	q31_t q31_u_d;
	q31_t q31_u_q;
	q31_t	sinevalue=0, cosinevalue=0;


	// Clark transformation
	arm_clarke_q31(int16_i_as, int16_i_bs, &q31_i_alpha, &q31_i_beta);
	arm_sin_cos_q31(q31_teta, &sinevalue, &cosinevalue);

	// Park transformation
	arm_park_q31(q31_i_alpha, q31_i_beta, &q31_i_d, &q31_i_q, sinevalue, cosinevalue);

	//Control iq
	q31_u_q =  PI_control_i_q(q31_i_q, int16_i_q_target);

	//Control id
	q31_u_d =  PI_control_i_q(q31_i_d, 0); //control direct current to zero


	//limit voltage in rotating frame, refer chapter 4.10.1 of UM1052
	q31_t	q31_u_abs = hypotf(q31_u_q,q31_u_d); //absolute value of U in static frame

	if (q31_u_abs > _U_MAX){
		q31_u_q *=_U_MAX/q31_u_abs;
		q31_u_d *=_U_MAX/q31_u_abs;
	}

	//inverse Park transformation
	arm_inv_park_q31(q31_u_d, q31_u_q, &q31_u_alpha, &q31_u_beta, sinevalue, cosinevalue);

	//call SVPWM calculation
	svpwm(q31_u_alpha, q31_u_beta);

}
//PI Control for quadrature current iq (torque)
q31_t PI_control_i_q (q31_t ist, q31_t soll)
{
  q31_t q31_p;
  static q31_t q31_q_i;
  static q31_t q31_q_dc=0;
  q31_p=((q31_t)soll - (q31_t)ist)*P_FACTOR_I_Q;
  q31_q_i+=((q31_t)soll - (q31_t)ist)*I_FACTOR_I_Q;
    //avoid too big steps in one loop run
  if (q31_p+q31_q_i>q31_q_dc+5) q31_q_dc+=5;
  else if  (q31_p+q31_q_i<q31_q_dc-5)q31_q_dc-=5;
  else q31_q_dc=q31_p+q31_q_i;

  return (q31_q_dc);
}

q31_t PI_control_i_d (q31_t ist, q31_t soll)
  {
    q31_t q31_p;
    static q31_t q31_d_i;
    static q31_t q31_d_dc=0;
    q31_p=((q31_t)soll - (q31_t)ist)*P_FACTOR_I_D;
    q31_d_i+=((q31_t)soll - (q31_t)ist)*I_FACTOR_I_D;
    if (q31_d_i<0)q31_d_i=0;
    //avoid too big steps in one loop run
    if (q31_p+q31_d_i>q31_d_dc+5) q31_d_dc+=5;
    else if  (q31_p+q31_d_i<q31_d_dc-5) q31_d_dc-=5;
    else q31_d_dc=q31_p+q31_d_i;

    return (q31_d_dc);
  }

void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta)	{ //you have to send angle teta in rad

//SVPWM according to chapter 4.9 of UM1052


	q31_t q31_U_alpha = (_SQRT3) *_T * q31_u_alpha;
	q31_t q31_U_beta = -_T * q31_u_beta;
	q31_t X = q31_U_beta;
	q31_t Y = (q31_U_alpha+q31_U_beta)/2;
	q31_t Z = (q31_U_beta-q31_U_alpha)/2;

	//Sector 1 & 4
	if ((Y>=0 && Z<0 && X>0)||(Y < 0 && Z>=0 && X<=0)){
		switchtime[0] = ((_T+X-Z)>>12) + (_T>>1); //right shift 11 for dividing by Umax (=2^11), right shift 1 for dividing by 2
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[1] - (X>>11);
	}

	//Sector 2 & 5
	if ((Y>=0 && Z>=0) || (Y<0 && Z<0) ){
		switchtime[0] = ((_T+Y-Z)>>12) + (_T>>1);
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[0] - (Y>>11);
	}

	//Sector 3 & 6
	if ((Y<0 && Z>=0 && X>0)||(Y >= 0 && Z<0 && X<=0)){
		switchtime[0] = ((_T+Y-X)>>12) + (_T>>1);
		switchtime[2] = switchtime[0] - (Y>>11);
		switchtime[1] = switchtime[2] + (X>>11);
	}


}

