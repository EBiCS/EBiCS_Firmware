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
long long	temp1;
long long	temp2;
q31_t	temp3;
q31_t	temp4;
q31_t	temp5;
q31_t	temp6;
q31_t z;
q31_t startup_counter=0;	//counter for start up routine

q31_t q31_i_q_fil = 0;
q31_t q31_i_d_fil = 0;
q31_t q31_u_d = 0;
q31_t q31_u_q = 0;
volatile long long fl_x1_obs=0;
volatile long long fl_x2_obs=0;
q31_t fl_e_alpha_obs;
q31_t fl_e_beta_obs;
q31_t e_log[300][6];

q31_t q31_erps_counter=10000;
q31_t q31_erps_filtered=5000;


q31_t q31_e_q_obs = 0;
q31_t q31_e_d_obs = 0;
q31_t q31_e_d_obs_fil = 0;

char PI_flag=0;
char Obs_flag=1;

uint16_t LUT_atan[101]={0,
		209,
		417,
		626,
		834,
		1042,
		1250,
		1458,
		1665,
		1872,
		2079,
		2285,
		2491,
		2697,
		2902,
		3106,
		3310,
		3513,
		3715,
		3917,
		4118,
		4318,
		4517,
		4716,
		4914,
		5110,
		5306,
		5501,
		5695,
		5888,
		6080,
		6271,
		6461,
		6649,
		6837,
		7023,
		7209,
		7393,
		7576,
		7757,
		7938,
		8117,
		8295,
		8472,
		8647,
		8821,
		8994,
		9165,
		9336,
		9504,
		9672,
		9838,
		10003,
		10167,
		10329,
		10490,
		10649,
		10807,
		10964,
		11119,
		11274,
		11426,
		11578,
		11728,
		11876,
		12024,
		12170,
		12314,
		12458,
		12600,
		12740,
		12880,
		13018,
		13154,
		13290,
		13424,
		13557,
		13688,
		13819,
		13948,
		14076,
		14202,
		14328,
		14452,
		14575,
		14696,
		14817,
		14936,
		15054,
		15171,
		15287,
		15402,
		15515,
		15628,
		15739,
		15849,
		15958,
		16066,
		16173,
		16279,
		16383
} ;


//const q31_t _T = 2048;

TIM_HandleTypeDef htim1;


void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target, MotorState_t* MS_FOC);
void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta);
q31_t PI_control_i_q (q31_t ist, q31_t soll);
q31_t PI_control_i_d (q31_t ist, q31_t soll);
q31_t atan2_LUT(q31_t e_alpha, q31_t e_beta);
void observer_update(long long v_alpha, long long v_beta, long long i_alpha, long long i_beta,  q31_t *e_alpha,q31_t *e_beta);
int utils_truncate_number_abs(long long *number, q31_t max);






void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, int16_t int16_i_q_target, MotorState_t* MS_FOC)
{

	 q31_t q31_i_alpha = 0;
	 q31_t q31_i_beta = 0;

	 q31_t q31_i_alpha_corr = 0;
	 q31_t q31_i_beta_corr = 0;

	 q31_t q31_u_alpha = 0;
	 q31_t q31_u_beta = 0;
	 q31_t q31_i_d = 0;
	 q31_t q31_i_q = 0;
	 static q31_t q31_angle_old = 0;


	 q31_t sinevalue=0, cosinevalue = 0;


	// temp5=(q31_t)int16_i_as;
	// temp6=(q31_t)int16_i_bs;

	// Clark transformation
	arm_clarke_q31((q31_t)int16_i_as, (q31_t)int16_i_bs, &q31_i_alpha, &q31_i_beta);

	arm_sin_cos_q31(q31_teta, &sinevalue, &cosinevalue);


	arm_park_q31(q31_i_alpha, q31_i_beta, &q31_i_alpha_corr, &q31_i_beta_corr, MS_FOC->sin_delay_filter,  MS_FOC->cos_delay_filter);

	// Park transformation
	arm_park_q31(q31_i_alpha_corr, q31_i_beta_corr, &q31_i_d, &q31_i_q, sinevalue, cosinevalue);


	q31_i_q_fil -= q31_i_q_fil>>4;
	q31_i_q_fil += q31_i_q;
	MS_FOC->i_q=q31_i_q_fil>>4;

	q31_i_d_fil -= q31_i_d_fil>>4;
	q31_i_d_fil += q31_i_d;
	MS_FOC->i_d=q31_i_d_fil>>4;

	if(q31_i_q>(PH_CURRENT_MAX<<2)){
		TIM1->BDTR &= ~(1L<<15);		//disable PWM if overcurrent detected
		while(1){}						//stay here until hard reset
	}
	//Control iq and id

	PI_flag=1;


if(!MS_FOC->Motor_state&&int16_i_q_target>0){

	MS_FOC->u_d=200;
	q31_teta_obs+=(2684354<<3);
	startup_counter++;
	if (startup_counter>5000){
		MS_FOC->Motor_state=1;
		startup_counter=0;
	}
}



	//inverse Park transformation
	arm_inv_park_q31(MS_FOC->u_d, MS_FOC->u_q, &q31_u_alpha, &q31_u_beta, -sinevalue, cosinevalue);

	//temp1= q31_i_q;
	//temp2= q31_i_alpha_corr;
	//temp3= q31_u_alpha;
    //temp4= q31_u_beta;



	observer_update(((long long)q31_u_alpha*(long long)adcData[0]*CAL_V)>>11, ((long long)(-q31_u_beta*(long long)adcData[0]*CAL_V))>>11, (long long)((-q31_i_alpha_corr)*CAL_I), (long long)((-q31_i_beta_corr)*CAL_I), &fl_e_alpha_obs, &fl_e_beta_obs);

if(MS_FOC->Motor_state){
	q31_teta_obs=atan2_LUT(-fl_e_beta_obs,fl_e_alpha_obs)-811271600;//-811271600;//-930576247;//-1431655765;
}
	if(q31_erps_counter<10000)q31_erps_counter++;
	else {
		MS_FOC->Speed=10000;
		MS_FOC->Motor_state=0;
		startup_counter=0;
	}
	//temp5=q31_erps_counter;
	if (q31_angle_old>(1<<25)&&q31_teta_obs<-(1<<25)&&q31_erps_counter>25){   //Find switch from +180° to -179,999° to detect one completed electric revolution.

		q31_erps_filtered-=q31_erps_filtered>>4;
		q31_erps_filtered+=q31_erps_counter;
		MS_FOC->Speed=q31_erps_filtered>>4;
		q31_erps_counter=0;
	}
	q31_angle_old=q31_teta_obs;



	temp5=fl_e_alpha_obs;
	temp6=fl_e_beta_obs;
	//temp3=q31_teta_obs>>24;
	//temp5=q31_teta>>24;
	//temp6=q31_teta_obs>>24;


	//temp1=int16_i_as;
	//temp2=int16_i_bs;
	if(!HAL_GPIO_ReadPin(PAS_GPIO_Port, PAS_Pin)&&ui8_debug_state==0)
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

	//q31_u_alpha=250;
	//q31_u_beta=0;
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



//PI Control for direct current id (loss)
q31_t PI_control_i_d (q31_t ist, q31_t soll)
  {
    q31_t q31_p;
    static q31_t q31_d_i = 0;
    static q31_t q31_d_dc = 0;

    q31_p=((soll - ist)*P_FACTOR_I_D)>>6;
    q31_d_i+=((soll - ist)*I_FACTOR_I_D)>>4;

    if (q31_d_i<-127)q31_d_i=-127;
    if (q31_d_i>127)q31_d_i=127;
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
void observer_update(long long v_alpha, long long v_beta, long long i_alpha, long long i_beta, q31_t *e_alpha, q31_t *e_beta) {

	const long long L = (3LL * INDUCTANCE)>>1;
	const long long lambda = FLUX_LINKAGE;
	long long R = (3LL * RESISTANCE)>>1;
	long long dT = 13LL;
	static long long x1=0;
	static long long x2=0;



/*
	// Saturation compensation
	const float sign = (m_motor_state.iq * m_motor_state.vq) >= 0.0 ? 1.0 : -1.0;
	R -= R * sign * m_conf->foc_sat_comp * (m_motor_state.i_abs_filter / m_conf->l_current_max);

	// Temperature compensation
	const float t = mc_interface_temp_motor_filtered();
	if (m_conf->foc_temp_comp && t > -5.0) {
		R += R * 0.00386 * (t - m_conf->foc_temp_comp_base_temp);
	}*/




	const long long L_ia = (L * i_alpha)>>16;//(iaf>>fact))>>5; // see comment in config.h for right shift
	const long long L_ib = (L * i_beta)>>16;//(ibf>>fact))>>5;
	const long long R_ia = (R * i_alpha)>>9;//(iaf>>fact))>>3;
	const long long R_ib = (R * i_beta)>>9;//(ibf>>fact))>>3;
	const long long lambda_2 = lambda*lambda;
	const long long gamma_half = GAMMA;
	//temp2=v_alpha;
	//temp1=R_ia;
	//temp2=L_ia;

	//temp1=i_alpha;
	//temp4=i_beta;
	//temp2=v_alpha;
	//temp4=v_beta;




	//temp1=v_alpha;
	//temp2=v_beta;

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
	*e_alpha= (x1 - L_ia);
	*e_beta=  (x2 - L_ib);
	//for (int i = 0;i <3;i++){
	// Same as above, but without iterations.
	long long err = lambda_2 - (((*e_alpha * *e_alpha)) + ((*e_beta * *e_beta)));

	long long gamma_tmp = gamma_half;
	if (utils_truncate_number_abs(&err, lambda_2>>0)) {
		//if(gamma_tmp>0) gamma_tmp--;
	}


	long long x1_dot = -R_ia + (v_alpha) + ((*e_alpha * err)>>gamma_tmp) ;
	long long x2_dot = -R_ib + (v_beta) + ((*e_beta * err)>>gamma_tmp) ;
	//temp1 = err;
	//temp2 = -R_ia + (v_alpha);
	x1 += x1_dot >>dT;
	x2 += x2_dot >>dT;

	//temp1 =-R_ia;
	//temp2 =v_alpha;
	temp3 =((*e_alpha * err)>>gamma_tmp);
	temp4 =err;


	*e_alpha= x1 - L_ia;// + (eaf>>24);
	*e_beta= x2 - L_ib;

	//temp3=*e_alpha;
	//temp4=err;



	//z++;
	//if (z>9)z=0;
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

q31_t atan2_LUT(q31_t e_alpha, q31_t e_beta){




uint8_t index =0;
uint8_t frac =0;
q31_t angle_obs=0;
uint8_t i=0;
//Quadrant 1 +
if (e_alpha>0 && e_beta>0){
    if(e_alpha>e_beta){  // y/x < 1

     index = (e_beta*100)/e_alpha;
     frac = (e_beta*1000/e_alpha)-10*index;
     angle_obs=LUT_atan[index]+(LUT_atan[index+1]-LUT_atan[index])*frac/10; //interpolate between to values
    }
     else {                     // y/x > 1 artan(y/x) = Pi/2 - artan(x/y)

     index = (e_alpha*100)/e_beta;
     frac = (e_alpha*1000/e_beta)-10*index;
         angle_obs= 32767-(LUT_atan[index]+((LUT_atan[index+1]-LUT_atan[index])*frac)/10);
     }
}
//Quadrant 2
if (e_alpha<0 && e_beta>0){

	if(-e_alpha>e_beta){  // y/x < 1
    index = (e_beta*100)/-e_alpha;
    frac = (e_beta*1000/-e_alpha)-10*index;
    angle_obs = 65535-(LUT_atan[index]+(LUT_atan[index+1]-LUT_atan[index])*frac/10); //interpolate between to values
   }
    else {                     // y/x > 1 artan(y/x) = Pi/2 - artan(x/y)

    index = (-e_alpha*100)/e_beta;
    frac = (-e_alpha*1000/e_beta)-10*index;
        angle_obs= 65535-(32767-(LUT_atan[index]+((LUT_atan[index+1]-LUT_atan[index])*frac)/10));
    }

}

//Quadrant 3
if (e_alpha<0 && e_beta<0){
    if(-e_alpha>-e_beta){  // y/x < 1

     index = (e_beta*100)/e_alpha;
     frac = (e_beta*1000/e_alpha)-10*index;
     angle_obs= -65535 + LUT_atan[index]+(LUT_atan[index+1]-LUT_atan[index])*frac/10; //interpolate between to values
    }
     else {                     // y/x > 1 artan(y/x) = Pi/2 - artan(x/y)

     index = (e_alpha*100)/e_beta;
     frac = (e_alpha*1000/e_beta)-10*index;
         angle_obs= -32767-(LUT_atan[index]+((LUT_atan[index+1]-LUT_atan[index])*frac)/10);
     }

}

//Quadrant 4
if (e_alpha>0 && e_beta<0){
    if(e_alpha>-e_beta){  // y/x < 1

     index = (-e_beta*100)/e_alpha;
     frac = (-e_beta*1000/e_alpha)-10*index;
     angle_obs=-(LUT_atan[index]+(LUT_atan[index+1]-LUT_atan[index])*frac/10); //interpolate between to values
    }
     else {                     // y/x > 1 artan(y/x) = Pi/2 - artan(x/y)

     index = (e_alpha*100)/-e_beta;
     frac = (e_alpha*1000/-e_beta)-10*index;
         angle_obs= -(32767-(LUT_atan[index]+((LUT_atan[index+1]-LUT_atan[index])*frac)/10));
     }

}
return ((angle_obs<<15)+1550960412); //angle in degree to q31 Look up table is scaled to 90° = 2^16 -1431655765
}

int utils_truncate_number_abs(long long *number, q31_t max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < -max) {
		*number = -max;
		did_trunc = 1;
	}

	return did_trunc;
}
