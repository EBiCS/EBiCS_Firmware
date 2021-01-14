/*
 * FOC.c
 *
 *  Created on: 25.01.2019
 *      Author: Stancecoke
 */
#include "main.h"
#include "config.h"
#include "FOC.h"
#include "stm32f1xx_hal.h"
#include <arm_math.h>

//q31_t	T_halfsample = 0.00003125;
//q31_t	counterfrequency = 64000000;
//q31_t	U_max = (1/_SQRT3)*_U_DC;
q31_t	temp1;
q31_t	temp2;
q31_t	temp3;
q31_t	temp4;
q31_t	temp5;
q31_t	temp6;

q31_t q31_i_q_fil = 0;
q31_t q31_i_d_fil = 0;

q31_t x1;
q31_t x2;
q31_t teta_obs;

q31_t e_log[300][6];
q31_t z;
char Obs_flag=1;
uint8_t ui8_debug_state=0;

char PI_flag=0;

//const q31_t _T = 2048;

TIM_HandleTypeDef htim1;


void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target, MotorState_t* MS_FOC);
void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta);
q31_t PI_control_i_q (q31_t ist, q31_t soll);
q31_t PI_control_i_d (q31_t ist, q31_t soll);
void observer_update(q31_t v_alpha, q31_t v_beta, q31_t i_alpha, q31_t i_beta, volatile q31_t x1, volatile q31_t x2, volatile q31_t phase);







void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target, MotorState_t* MS_FOC)
{

	 q31_t q31_i_alpha = 0;
	 q31_t q31_i_beta = 0;
	 q31_t q31_u_alpha = 0;
	 q31_t q31_u_beta = 0;
	 q31_t q31_i_d = 0;
	 q31_t q31_i_q = 0;

	 q31_t sinevalue=0, cosinevalue = 0;


	// temp5=(q31_t)int16_i_as;
	// temp6=(q31_t)int16_i_bs;

	// Clark transformation
	arm_clarke_q31((q31_t)int16_i_as, (q31_t)int16_i_bs, &q31_i_alpha, &q31_i_beta);

	arm_sin_cos_q31(q31_teta, &sinevalue, &cosinevalue);


	// Park transformation
	arm_park_q31(q31_i_alpha, q31_i_beta, &q31_i_d, &q31_i_q, sinevalue, cosinevalue);


	q31_i_q_fil -= q31_i_q_fil>>4;
	q31_i_q_fil += q31_i_q;
	MS_FOC->i_q=q31_i_q_fil>>4;

	q31_i_d_fil -= q31_i_d_fil>>4;
	q31_i_d_fil += q31_i_d;
	MS_FOC->i_d=q31_i_d_fil>>4;

	//Control iq

	PI_flag=1;

//set static volatage for hall angle detection
if(!MS_FOC->hall_angle_detect_flag){
	MS_FOC->u_q=0;
	MS_FOC->u_d=200;
	}


	//inverse Park transformation
	arm_inv_park_q31(MS_FOC->u_d, MS_FOC->u_q, &q31_u_alpha, &q31_u_beta, -sinevalue, cosinevalue);
/*
	temp1=int16_i_as;
	temp2=int16_i_bs;
	temp3=MS_FOC->i_d;
	temp4=MS_FOC->i_q;
	temp5=MS_FOC->u_d;
	temp6=MS_FOC->u_q;
*/


	if(uint32_PAS_counter < PAS_TIMEOUT&&ui8_debug_state==0)
			{
		e_log[z][0]=temp1;//fl_e_alpha_obs;
		e_log[z][1]=temp2;//fl_e_beta_obs;
		e_log[z][2]=temp3;//(q31_t)q31_teta_obs>>24;
		e_log[z][3]=temp4;
		e_log[z][4]=temp5;
		e_log[z][5]=temp6;
		z++;
		if(z>150) Obs_flag=1;
		if (z>299)
		{z=0;

		ui8_debug_state=2;}
			}
	else {if(ui8_debug_state==2)ui8_debug_state=3;;}

	//call SVPWM calculation
	svpwm(q31_u_alpha, q31_u_beta);
	//temp6=__HAL_TIM_GET_COUNTER(&htim1);

}
//PI Control for quadrature current iq (torque)
q31_t PI_control_i_q (q31_t ist, q31_t soll)
{

  q31_t q31_p; //proportional part
  static q31_t q31_q_i = 0; //integral part
  static q31_t q31_q_dc = 0; // sum of proportional and integral part
  q31_p = ((soll - ist)*P_FACTOR_I_Q);
  q31_q_i += ((soll - ist)*I_FACTOR_I_Q);
  temp5=q31_p;
  temp6=q31_q_i;

  if (q31_q_i>_U_MAX<<10) q31_q_i=_U_MAX<<10;
  if (q31_q_i<-_U_MAX<<10) q31_q_i = -_U_MAX<<10;
  if(!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE))q31_q_i = 0 ; //reset integral part if PWM is disabled

    //avoid too big steps in one loop run
  if ((q31_p+q31_q_i)>>10>q31_q_dc+5) q31_q_dc+=5;
  else if  ((q31_p+q31_q_i)>>10<q31_q_dc-5)q31_q_dc-=5;
  else q31_q_dc=(q31_p+q31_q_i)>>10;


  if (q31_q_dc>_U_MAX) q31_q_dc = _U_MAX;
  if (q31_q_dc<-_U_MAX) q31_q_dc = -_U_MAX; // allow no negative voltage.


  return (q31_q_dc);
}

//PI Control for direct current id (loss)
q31_t PI_control_i_d (q31_t ist, q31_t soll)
  {
    q31_t q31_p;
    static q31_t q31_d_i = 0;
    static q31_t q31_d_dc = 0;

    q31_p=((soll - ist)*P_FACTOR_I_D)>>5;
    q31_d_i+=((soll - ist)*I_FACTOR_I_D)>>5;

    if (q31_d_i<-1800)q31_d_i=-1800;
    if (q31_d_i>1800)q31_d_i=1800;

    if(!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE))q31_d_i = 0 ; //reset integral part if PWM is disabled
    //avoid too big steps in one loop run
    if (q31_p+q31_d_i>q31_d_dc+5) q31_d_dc+=5;
    else if  (q31_p+q31_d_i<q31_d_dc-5) q31_d_dc-=5;
    else q31_d_dc=q31_p+q31_d_i;

    if (q31_d_dc>_U_MAX) q31_d_dc = _U_MAX;
    if (q31_d_dc<-(_U_MAX)) q31_d_dc =- (_U_MAX);

    return (q31_d_dc);
  }

void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta)	{

//SVPWM according to chapter 4.9 of UM1052


	q31_t q31_U_alpha = (_SQRT3 *_T * q31_u_alpha)>>4;
	q31_t q31_U_beta = -_T * q31_u_beta;
	q31_t X = q31_U_beta;
	q31_t Y = (q31_U_alpha+q31_U_beta)>>1;
	q31_t Z = (q31_U_beta-q31_U_alpha)>>1;

	//Sector 1 & 4
	if ((Y>=0 && Z<0 && X>0)||(Y < 0 && Z>=0 && X<=0)){
		switchtime[0] = ((_T+X-Z)>>12) + (_T>>1); //right shift 11 for dividing by peroid (=2^11), right shift 1 for dividing by 2
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[1] - (X>>11);
		//temp4=1;
	}

	//Sector 2 & 5
	if ((Y>=0 && Z>=0) || (Y<0 && Z<0) ){
		switchtime[0] = ((_T+Y-Z)>>12) + (_T>>1);
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[0] - (Y>>11);
		//temp4=2;
	}

	//Sector 3 & 6
	if ((Y<0 && Z>=0 && X>0)||(Y >= 0 && Z<0 && X<=0)){
		switchtime[0] = ((_T+Y-X)>>12) + (_T>>1);
		switchtime[2] = switchtime[0] - (Y>>11);
		switchtime[1] = switchtime[2] + (X>>11);
		//temp4=3;
	}


}


