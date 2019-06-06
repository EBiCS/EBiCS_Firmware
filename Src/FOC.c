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
q31_t q31_u_d = 0;
q31_t q31_u_q = 0;
volatile static float fl_x1_obs;
volatile static float fl_x2_obs;
volatile static float fl_e_alpha_obs;
volatile static float fl_e_beta_obs;



q31_t q31_e_q_obs = 0;
q31_t q31_e_d_obs = 0;

char PI_flag=0;
char Obs_flag=0;

//const q31_t _T = 2048;

TIM_HandleTypeDef htim1;


void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target);
void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta);
q31_t PI_control_i_q (q31_t ist, q31_t soll);
q31_t PI_control_i_d (q31_t ist, q31_t soll);
q31_t PI_control_e_d (q31_t ist, q31_t soll);
void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, volatile float *x1, volatile float *x2, volatile float *e_alpha, volatile float *e_beta);







void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target)
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


	q31_i_q_fil -= q31_i_q_fil>>3;
	q31_i_q_fil += q31_i_q;
	q31_i_d_fil -= q31_i_d_fil>>3;
	q31_i_d_fil += q31_i_d;


	//Control iq

	PI_flag=1;

	/*
	q31_u_q =  PI_control_i_q(q31_i_q_fil>>3, (q31_t) int16_i_q_target);



	//Control id
	q31_u_d = -PI_control_i_d(q31_i_d_fil>>3, 0); //control direct current to zero

	//limit voltage in rotating frame, refer chapter 4.10.1 of UM1052

	q31_t	q31_u_abs = hypot(q31_u_q, q31_u_d); //absolute value of U in static frame
	temp3 = q31_u_abs;


	if (q31_u_abs > _U_MAX){
		q31_u_q = (q31_u_q*_U_MAX)/q31_u_abs; //division!
		q31_u_d = (q31_u_d*_U_MAX)/q31_u_abs; //division!
		temp4=1;
	}
	else temp4=0;
	*/

	//q31_u_q=0;
	//q31_u_d=0;
	//arm_sin_cos_q31(q31_teta, &sinevalue, &cosinevalue);
	//inverse Park transformation
	arm_inv_park_q31(q31_u_d, q31_u_q, &q31_u_alpha, &q31_u_beta, -sinevalue, cosinevalue);
/*
	temp1= q31_i_alpha;
	temp2= q31_i_beta;
	temp3= q31_u_alpha;
    temp4= q31_u_beta;
    */

	//>>11 because of 2048 max dutycycle, voltage values in mV, amps in mA
	observer_update((float)q31_u_alpha*(float)adcData[0]*CAL_V, (-(float)q31_u_beta*(float)adcData[0]*CAL_V), (float)q31_i_alpha*CAL_I, (float)q31_i_beta*CAL_I , &fl_x1_obs, &fl_x2_obs, &fl_e_alpha_obs, &fl_e_beta_obs);

	arm_park_q31((q31_t)fl_e_alpha_obs, (q31_t)fl_e_beta_obs, &q31_e_d_obs, &q31_e_q_obs, sinevalue, cosinevalue);

	Obs_flag=1;


	//call SVPWM calculation
	svpwm(q31_u_alpha, q31_u_beta);
	//temp6=__HAL_TIM_GET_COUNTER(&htim1);

}
//PI Control for quadrature current iq (torque) float operation without division
q31_t PI_control_i_q (q31_t ist, q31_t soll)
{

  q31_t q31_p; //proportional part
  static float flt_q_i = 0; //integral part
  static q31_t q31_q_dc = 0; // sum of proportional and integral part
  q31_p = (soll - ist)*P_FACTOR_I_Q;
  flt_q_i += ((float)(soll - ist))*I_FACTOR_I_Q;

  if ((q31_t)flt_q_i>_U_MAX) flt_q_i=(float)_U_MAX;
  if ((q31_t)flt_q_i<0) flt_q_i = 0 ;

    //avoid too big steps in one loop run
  if (q31_p+(q31_t)flt_q_i>q31_q_dc+5) q31_q_dc+=5;
  else if  (q31_p+(q31_t)flt_q_i<q31_q_dc-5)q31_q_dc-=5;
  else q31_q_dc=q31_p+(q31_t)flt_q_i;


  if (q31_q_dc>_U_MAX) q31_q_dc = _U_MAX;
  if (q31_q_dc<0) q31_q_dc = 0; // allow no negative voltage.

  return (q31_q_dc);
}

//PI Control for d-fraction of BEMF, sensorless commutation
q31_t PI_control_e_d (q31_t ist, q31_t soll)
{

  q31_t q31_p; //proportional part
  static float flt_q_i = 0; //integral part
  static q31_t q31_q_dc = 0; // sum of proportional and integral part
  q31_p = (soll - ist)*P_FACTOR_E_D;
  flt_q_i += ((float)(soll - ist))*I_FACTOR_E_D;

  q31_q_dc=q31_p+(q31_t)flt_q_i;

  return (q31_q_dc);
}

//PI Control for direct current id (loss)
q31_t PI_control_i_d (q31_t ist, q31_t soll)
  {
    q31_t q31_p;
    static q31_t q31_d_i = 0;
    static q31_t q31_d_dc = 0;

    q31_p=((soll - ist)*P_FACTOR_I_D)>>4;
    q31_d_i+=((soll - ist)*I_FACTOR_I_D)>>4;

    if (q31_d_i<-127)q31_d_i=-127;
    if (q31_d_i>127)q31_d_i=127;
    //avoid too big steps in one loop run
    if (q31_p+q31_d_i>q31_d_dc+5) q31_d_dc+=5;
    else if  (q31_p+q31_d_i<q31_d_dc-5) q31_d_dc-=5;
    else q31_d_dc=q31_p+q31_d_i;

    if (q31_d_dc>_U_MAX>>2) q31_d_dc = _U_MAX>>2;
    if (q31_d_dc<-(_U_MAX>>2)) q31_d_dc =- (_U_MAX>>2);

    return (q31_d_dc);
  }

void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta)	{

//SVPWM according to chapter 4.9 of UM1052


	q31_t q31_U_alpha = (q31_t)((float)_SQRT3 *(float)_T * (float) q31_u_alpha); //float operation to avoid q31 overflow
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

// See http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, volatile float *x1, volatile float *x2, volatile float *e_alpha, volatile float *e_beta) {

	const float L = (3.0 / 2.0) * INDUCTANCE;
	const float lambda = FLUX_LINKAGE;
	float R = (3.0 / 2.0) * RESISTANCE;
	float dT = 1/(2000.0);
/*
	// Saturation compensation
	const float sign = (m_motor_state.iq * m_motor_state.vq) >= 0.0 ? 1.0 : -1.0;
	R -= R * sign * m_conf->foc_sat_comp * (m_motor_state.i_abs_filter / m_conf->l_current_max);

	// Temperature compensation
	const float t = mc_interface_temp_motor_filtered();
	if (m_conf->foc_temp_comp && t > -5.0) {
		R += R * 0.00386 * (t - m_conf->foc_temp_comp_base_temp);
	}*/

	const float L_ia = (L * i_alpha); // diveded by 1000 because of value in milliamps
	const float L_ib = (L * i_beta);
	const float R_ia = (R * i_alpha);
	const float R_ib = (R * i_beta);
	const float lambda_2 = lambda*lambda;
	const float gamma_half = GAMMA * 0.5;

	// Original
//	float err = lambda_2 - (SQ(*x1 - L_ia) + SQ(*x2 - L_ib));
//	float x1_dot = -R_ia + v_alpha + gamma_half * (*x1 - L_ia) * err;
//	float x2_dot = -R_ib + v_beta + gamma_half * (*x2 - L_ib) * err;
//	*x1 += x1_dot * dt;
//	*x2 += x2_dot * dt;
/*
	// Iterative with some trial and error
	const int iterations = 6;
	const float dt_iteration = dt / (float)iterations;
	for (int i = 0;i < iterations;i++) {
		float err = lambda_2 - (SQ(*x1 - L_ia) + SQ(*x2 - L_ib));
		float gamma_tmp = gamma_half;
		if (utils_truncate_number_abs(&err, lambda_2 * 0.2)) {
			gamma_tmp *= 10.0;
		}
		float x1_dot = -R_ia + v_alpha + gamma_tmp * (*x1 - L_ia) * err;
		float x2_dot = -R_ib + v_beta + gamma_tmp * (*x2 - L_ib) * err;

		*x1 += x1_dot * dt_iteration;
		*x2 += x2_dot * dt_iteration;
	}
	*/
	//*x1=0;
	//*x2=0;
	//for (int i = 0;i <3;i++){
	// Same as above, but without iterations.
	volatile q31_t err = lambda_2 - ((*x1 - L_ia)*(*x1 - L_ia) + (*x2 - L_ib)*(*x2 - L_ib));
	float gamma_tmp = gamma_half;
	/*if (utils_truncate_number_abs(&err, lambda_2 * 0.2)) {
		gamma_tmp *= 10.0;
	}*/


	volatile q31_t x1_dot = -R_ia + v_alpha + (((*x1 - L_ia) * err)*gamma_tmp) ;
	volatile q31_t x2_dot = -R_ib + v_beta + (((*x2 - L_ib) * err)*gamma_tmp) ;

	*x1 += x1_dot *dT;
	*x2 += x2_dot *dT;

	//}

	*e_alpha= *x1 - L_ia;
	*e_beta= *x2 - L_ib;
	temp1=(q31_t)(*e_alpha*10000.0);
	temp2=(q31_t)(*e_beta*10000.0);
	if((q31_t)v_alpha>temp3)temp3=(q31_t)v_alpha;
	if((q31_t)v_alpha<temp4)temp4=(q31_t)v_alpha;
	if((q31_t)R_ia>temp5)temp5=(q31_t)R_ia;
	if((q31_t)R_ia<temp6)temp6=(q31_t)R_ia;


	//UTILS_NAN_ZERO(*x1);
	//UTILS_NAN_ZERO(*x2);

	//*phase = utils_fast_atan2(*x2 - L_ib, *x1 - L_ia);
}
/*
static void pll_run(float phase, float dt, volatile float *phase_var,volatile float *speed_var) {
	//UTILS_NAN_ZERO(*phase_var);
	float delta_theta = phase - *phase_var;
	//utils_norm_angle_rad(&delta_theta);
	//UTILS_NAN_ZERO(*speed_var);
	*phase_var += (*speed_var + SPEED_KP * delta_theta) * dt;
	//utils_norm_angle_rad((float*)phase_var);
	*speed_var += SPEED_KI * delta_theta * dt;
}*/
