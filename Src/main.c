
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"




/* USER CODE BEGIN Includes */

// Lishui BLCD FOC Open Source Firmware
// Board uses IRS2003 half bridge drivers, this need inverted pulses for low-side Mosfets, deadtime is generated in driver
// This firmware bases on the ST user manual UM1052
// It uses the OpenSTM32 workbench (SW4STM32 toolchain)
// Basic peripheral setup was generated with CubeMX

#include "print.h"
#include "FOC.h"
#include "config.h"
#include "eeprom.h"


#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
#include "display_kingmeter.h"
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
#include "display_bafang.h"
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
#include "display_kunteng.h"
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_EBiCS)
#include "display_ebics.h"
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
#include "display_No_2.h"
#endif

#include <arm_math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


uint32_t ui32_tim1_counter=0;
uint32_t ui32_tim3_counter=0;
uint8_t ui8_hall_state=0;
uint8_t ui8_hall_state_old=0;
uint8_t ui8_hall_case =0;
uint16_t ui16_tim2_recent=0;
uint16_t ui16_timertics=5000; 					//timertics between two hall events for 60Â° interpolation
uint16_t ui16_throttle;
uint16_t ui16_throttle_offset = THROTTLE_OFFSET;
uint16_t ui16_brake_adc;
uint32_t ui32_throttle_cumulated;
uint32_t ui32_brake_adc_cumulated;
uint32_t ui32_int_Temp_cumulated = INT_TEMP_25<<5;
int16_t i16_int_Temp_V25;
uint16_t ui16_ph1_offset=0;
uint16_t ui16_ph2_offset=0;
uint16_t ui16_ph3_offset=0;
int16_t i16_ph1_current=0;

int16_t i16_ph2_current=0;
int16_t i16_ph2_current_filter=0;
int16_t i16_ph3_current=0;
uint16_t i=0;
uint16_t j=0;
uint16_t k=0;
uint16_t y=0;
uint8_t brake_flag=0;
volatile uint8_t ui8_overflow_flag=0;
uint8_t ui8_slowloop_counter=0;
volatile uint8_t ui8_adc_inj_flag=0;
volatile uint8_t ui8_adc_regular_flag=0;
uint8_t ui8_speedcase=0;
uint8_t ui8_speedfactor=0;
int8_t i8_direction= REVERSE; //for permanent reverse direction
int8_t i8_reverse_flag = 1; //for temporaribly reverse direction
uint8_t ui8_KV_detect_flag = 0; //for getting the KV of the motor after auto angle detect
uint16_t ui16_KV_detect_counter = 0; //for getting timing of the KV detect
int16_t ui32_KV = 0;


volatile uint8_t ui8_adc_offset_done_flag=0;
volatile uint8_t ui8_print_flag=0;
volatile uint8_t ui8_UART_flag=0;
volatile uint8_t ui8_Push_Assist_flag=0;
volatile uint8_t ui8_UART_TxCplt_flag=1;
volatile uint8_t ui8_PAS_flag=0;
volatile uint8_t ui8_SPEED_flag=0;
volatile uint8_t ui8_SPEED_control_flag=0;
volatile uint8_t ui8_BC_limit_flag=0;  //flag for Battery current limitation
volatile uint8_t ui8_6step_flag=0;
uint32_t uint32_PAS_counter= PAS_TIMEOUT+1;
uint32_t uint32_PAS_HIGH_counter= 0;
uint32_t uint32_PAS_HIGH_accumulated= 32000;
uint32_t uint32_PAS_fraction= 100;
uint32_t uint32_SPEED_counter=32000;
uint32_t uint32_SPEEDx100_cumulated=0;
uint32_t uint32_PAS=32000;

q31_t q31_rotorposition_PLL = 0;
q31_t q31_angle_per_tic = 0;

uint8_t ui8_UART_Counter=0;
int8_t i8_recent_rotor_direction=1;
int16_t i16_hall_order=1;
uint16_t ui16_erps=0;

uint32_t uint32_torque_cumulated=0;
uint32_t uint32_PAS_cumulated=32000;
uint16_t uint16_mapped_throttle=0;
uint16_t uint16_mapped_PAS=0;
uint16_t uint16_mapped_BRAKE=0;
uint16_t uint16_half_rotation_counter=0;
uint16_t uint16_full_rotation_counter=0;
int32_t int32_temp_current_target=0;
q31_t q31_PLL_error=0;
q31_t q31_t_Battery_Current_accumulated=0;

q31_t q31_rotorposition_absolute;
q31_t q31_rotorposition_hall;
//q31_t q31_rotorposition_motor_specific = SPEC_ANGLE;
q31_t q31_u_d_temp=0;
q31_t q31_u_q_temp=0;
int16_t i16_sinus=0;
int16_t i16_cosinus=0;
char buffer[100];
char char_dyn_adc_state_old=1;
const uint8_t assist_factor[10]={0, 51, 102, 153, 204, 255, 255, 255, 255, 255};
const uint8_t assist_profile[2][6]= {	{0,10,20,30,45,48},
		{64,64,128,200,255,0}};

uint16_t switchtime[3];
volatile uint16_t adcData[9]; //Buffer for ADC1 Input
q31_t tic_array[6];

//Rotor angle scaled from degree to q31 for arm_math. -180Â°-->-2^31, 0Â°-->0, +180Â°-->+2^31
const q31_t deg_30 = 357913941;

q31_t Hall_13 = 0;
q31_t Hall_32 = 0;
q31_t Hall_26 = 0;
q31_t Hall_64 = 0;
q31_t Hall_51 = 0;
q31_t Hall_45 = 0;

const q31_t tics_lower_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*SPEEDLIMIT*10); //tics=wheelcirc*timerfrequency/(no. of hallevents per rev*gear-ratio*speedlimit)*3600/1000000
const q31_t tics_higher_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*(SPEEDLIMIT+2)*10);
uint32_t uint32_tics_filtered=1000000;

uint16_t VirtAddVarTab[NB_OF_VAR] = { 	EEPROM_POS_HALL_ORDER,
		EEPROM_POS_HALL_45,
		EEPROM_POS_HALL_51,
		EEPROM_POS_HALL_13,
		EEPROM_POS_HALL_32,
		EEPROM_POS_HALL_26,
		EEPROM_POS_HALL_64,
		EEPROM_INT_TEMP_V25
};

enum state {Stop, SixStep, Regen, Running, BatteryCurrentLimit, Interpolation, PLL, IdleRun};
enum state SystemState;

#define iabs(x) (((x) >= 0)?(x):-(x))
#define sign(x) (((x) >= 0)?(1):(-1))


//variables for display communication
#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
KINGMETER_t KM;
#endif

//variables for display communication
#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
BAFANG_t BF;
#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_EBiCS)
uint8_t ui8_main_LEV_Page_counter=0;
uint8_t ui8_additional_LEV_Page_counter=0;
uint8_t ui8_LEV_Page_to_send=1;
#endif

//variables for display communication
#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
No2_t No2;
#endif

MotorState_t MS;
MotorParams_t MP;

//structs for PI_control
PI_control_t PI_iq;
PI_control_t PI_id;
PI_control_t PI_speed;


int16_t battery_percent_fromcapacity = 50; 			//Calculation of used watthours not implemented yet
int16_t wheel_time = 1000;							//duration of one wheel rotation for speed calculation
int16_t current_display;							//pepared battery current for display

int16_t power;										//recent power output

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
int16_t T_NTC(uint16_t ADC);
void init_watchdog(void);
void MX_IWDG_Init(void);
void get_internal_temp_offset(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER|| DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
void kingmeter_update(void);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
void bafang_update(void);
#endif

static void dyn_adc_state(q31_t angle);
static void set_inj_channel(char state);
void get_standstill_position();
q31_t speed_PLL (q31_t ist, q31_t soll, uint8_t speedadapt);
int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t speed_to_tics (uint8_t speed);
int8_t tics_to_speed (uint32_t tics);
int16_t internal_tics_to_speedx100 (uint32_t tics);
int16_t external_tics_to_speedx100 (uint32_t tics);





/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */



	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();


	//initialize MS struct.
	MS.hall_angle_detect_flag=1;
	MS.Speed=128000;
	MS.assist_level=127;
	MS.regen_level=7;
	MS.i_q_setpoint = 0;
	MS.i_d_setpoint = 0;
	MS.angle_est=SPEED_PLL;


	MP.pulses_per_revolution = PULSES_PER_REVOLUTION;
	MP.wheel_cirumference = WHEEL_CIRCUMFERENCE;
	MP.speedLimit=SPEEDLIMIT;
	MP.battery_current_max = BATTERYCURRENT_MAX;


	//init PI structs
	PI_id.gain_i=I_FACTOR_I_D;
	PI_id.gain_p=P_FACTOR_I_D;
	PI_id.setpoint = 0;
	PI_id.limit_output = _U_MAX;
	PI_id.max_step=5000;
	PI_id.shift=10;
	PI_id.limit_i=1800;

	PI_iq.gain_i=I_FACTOR_I_Q;
	PI_iq.gain_p=P_FACTOR_I_Q;
	PI_iq.setpoint = 0;
	PI_iq.limit_output = _U_MAX;
	PI_iq.max_step=5000;
	PI_iq.shift=10;
	PI_iq.limit_i=_U_MAX;

#ifdef SPEEDTHROTTLE

	PI_speed.gain_i=I_FACTOR_SPEED;
	PI_speed.gain_p=P_FACTOR_SPEED;
	PI_speed.setpoint = 0;
	PI_speed.limit_output = PH_CURRENT_MAX;
	PI_speed.max_step=50;
	PI_speed.shift=5;
	PI_speed.limit_i=PH_CURRENT_MAX;

#endif

	//Virtual EEPROM init
	HAL_FLASH_Unlock();
	EE_Init();
	HAL_FLASH_Lock();

	MX_ADC1_Init();
	/* Run the ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}
	MX_ADC2_Init();
	/* Run the ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}

	/* USER CODE BEGIN 2 */
	SET_BIT(ADC1->CR2, ADC_CR2_JEXTTRIG);//external trigger enable
	__HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);
	SET_BIT(ADC2->CR2, ADC_CR2_JEXTTRIG);//external trigger enable
	__HAL_ADC_ENABLE_IT(&hadc2,ADC_IT_JEOC);


	//HAL_ADC_Start_IT(&hadc1);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcData, 8);
	HAL_ADC_Start_IT(&hadc2);
	MX_TIM1_Init(); //Hier die Reihenfolge getauscht!
	MX_TIM2_Init();
	MX_TIM3_Init();

	// Start Timer 1
	if(HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
	{
		/* Counter Enable Error */
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);




	TIM1->CCR4 = TRIGGER_DEFAULT; //ADC sampling just before timer overflow (just before middle of PWM-Cycle)
	//PWM Mode 1: Interrupt at counting down.

	//TIM1->BDTR |= 1L<<15;
	// TIM1->BDTR &= ~(1L<<15); //reset MOE (Main Output Enable) bit to disable PWM output
	// Start Timer 2
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

	// Start Timer 3

	if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	{
		/* Counter Enable Error */
		Error_Handler();
	}


#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER || DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
	KingMeter_Init (&KM);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
	Bafang_Init (&BF);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
	kunteng_init();
	check_message(&MS, &MP);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_EBiCS)
	//  ebics_init();
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
	No2_Init(&No2);
#endif
	TIM1->CCR1 = 1023; //set initial PWM values
	TIM1->CCR2 = 1023;
	TIM1->CCR3 = 1023;


	CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);//Disable PWM

	HAL_Delay(500); //wait for stable conditions

	for(i=0;i<32;i++){
		while(!ui8_adc_regular_flag){}
		temp1+=adcData[2];
		temp2+=adcData[3];
		temp3+=adcData[4];
		ui8_adc_regular_flag=0;

#ifdef TQONAD1
		temp4+=adcData[6];
#else
		temp4+=adcData[1];
#endif
	}
	ui16_ph1_offset=temp1>>5;
	ui16_ph2_offset=temp2>>5;
	ui16_ph3_offset=temp3>>5;
	ui16_throttle_offset=(temp4>>5)+5;

#ifdef DISABLE_DYNAMIC_ADC // set  injected channel with offsets
	ADC1->JSQR=0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
	ADC1->JOFR1 = ui16_ph1_offset;
	ADC2->JSQR=0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
	ADC2->JOFR1 = ui16_ph2_offset;
#endif

	ui8_adc_offset_done_flag=1;
	init_watchdog();

#if defined (ADC_BRAKE)

	while ((adcData[5]>ui16_throttle_offset)&&(adcData[1]>(THROTTLE_MAX-ui16_throttle_offset))){HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);
	y++;
	if(y==35) autodetect();
	}

#endif

	//run autodect, whenn brake is pulled an throttle is pulled for 10 at startup
#ifndef NCTE

	while ((!HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin))&&(adcData[1]>(ui16_throttle_offset+20))){
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		y++;
		if(y==35) autodetect();
	}
#else
	ui32_throttle_cumulated=ui16_throttle_offset<<4;
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
	printf_("phase current offsets:  %d, %d, %d \n ", ui16_ph1_offset, ui16_ph2_offset, ui16_ph3_offset);
	printf_("internal temperature raw reading:  %d, \n ", adcData[7]);
#if (AUTODETECT == 1)
	if(adcData[0]>VOLTAGE_MIN) autodetect();
	else printf_("Battery voltage too low!:  %d,\n ",adcData[0]);
#endif

#endif

#ifdef NCTE
	while(adcData[1]<ui16_throttle_offset)
#else
		// 	while(adcData[1]>ui16_throttle_offset)
#endif
	{
		HAL_IWDG_Refresh(&hiwdg);//do nothing (For Safety at switching on)
	}
	//read internal temp calibration from emulated EEPROM
	EE_ReadVariable(EEPROM_INT_TEMP_V25, &i16_int_Temp_V25);
	if(!i16_int_Temp_V25)i16_int_Temp_V25=INT_TEMP_25; //use value from main.h, if not in EEPROM yet.

#if (!USE_FIX_POSITIONS)
	EE_ReadVariable(EEPROM_POS_HALL_ORDER, &i16_hall_order);
	printf_("Hall_Order: %d \n",i16_hall_order);
	// set varaiables to value from emulated EEPROM only if valid
	if(i16_hall_order!=0xFFFF) {
		int16_t temp;

		EE_ReadVariable(EEPROM_POS_HALL_45, &temp);
		Hall_45 = temp<<16;
		printf_("Hall_45: %d \n",	(int16_t) (((Hall_45 >> 23) * 180) >> 8));
		printf_("Hall_45: %u \n",	Hall_45);
		HAL_IWDG_Refresh(&hiwdg);

		EE_ReadVariable(EEPROM_POS_HALL_51, &temp);
		Hall_51 = temp<<16;
		printf_("Hall_51: %d \n",	(int16_t) (((Hall_51 >> 23) * 180) >> 8));
		printf_("Hall_51: %u \n",	Hall_51);
		HAL_IWDG_Refresh(&hiwdg);

		EE_ReadVariable(EEPROM_POS_HALL_13, &temp);
		Hall_13 = temp<<16;
		printf_("Hall_13: %d \n",	(int16_t) (((Hall_13 >> 23) * 180) >> 8));
		printf_("Hall_13: %u \n",	Hall_13);
		HAL_IWDG_Refresh(&hiwdg);

		EE_ReadVariable(EEPROM_POS_HALL_32, &temp);
		Hall_32 = temp<<16;
		printf_("Hall_32: %d \n",	(int16_t) (((Hall_32 >> 23) * 180) >> 8));
		printf_("Hall_32: %u \n",	Hall_32);
		HAL_IWDG_Refresh(&hiwdg);

		EE_ReadVariable(EEPROM_POS_HALL_26, &temp);
		Hall_26 = temp<<16;
		printf_("Hall_26: %d \n",	(int16_t) (((Hall_26 >> 23) * 180) >> 8));
		printf_("Hall_26: %u \n",	Hall_26);
		HAL_IWDG_Refresh(&hiwdg);

		EE_ReadVariable(EEPROM_POS_HALL_64, &temp);
		Hall_64 = temp<<16;
		printf_("Hall_64: %d \n",	(int16_t) (((Hall_64 >> 23) * 180) >> 8));
		printf_("Hall_64: %u \n",	Hall_64);
		HAL_IWDG_Refresh(&hiwdg);

		EE_ReadVariable(EEPROM_POS_KV, &ui32_KV);
		if(!ui32_KV)ui32_KV=111;
		printf_("KV: %d \n",ui32_KV	);
		HAL_IWDG_Refresh(&hiwdg);

	}

#else
	i16_hall_order = HALL_ORDER;
	ui32_KV = KV;
	Hall_45 = HALL_45;
	Hall_51 = HALL_51;
	Hall_13 = HALL_13;
	Hall_32 = HALL_32;
	Hall_26 = HALL_26;
	Hall_64 = HALL_64;
#endif


	// set absolute position to corresponding hall pattern.

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
	printf_("Lishui FOC v1.0 \n ");

#endif


	CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);//Disable PWM

	get_standstill_position();


	//	init_watchdog();


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg);


		/* if(PI_flag){
	  runPIcontrol();
	  PI_flag=0;
	  }*/
		//display message processing
		if(ui8_UART_flag){
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U||DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)

			KingMeter_Service(&KM);
#endif
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
			kingmeter_update();

#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
			bafang_update();
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
			check_message(&MS, &MP);
			if(MS.assist_level==6)ui8_Push_Assist_flag=1;
			else ui8_Push_Assist_flag=0;
#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_EBiCS)
			//  process_ant_page(&MS, &MP);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
			No2_Service(&No2);
#endif
			ui8_UART_flag=0;
		}


		//process regualr ADC
		if(ui8_adc_regular_flag){
			ui32_throttle_cumulated -= ui32_throttle_cumulated>>4;
#ifdef TQONAD1
			ui32_throttle_cumulated += adcData[6]; //get value from AD1 PB1
#else
			ui32_throttle_cumulated += adcData[1]; //get value from SP
#endif
			ui32_brake_adc_cumulated -= ui32_brake_adc_cumulated>>4;
			ui32_brake_adc_cumulated+=adcData[5];//get value for analog brake from AD2 = PB0
			ui16_brake_adc=ui32_brake_adc_cumulated>>4;
			ui16_throttle = ui32_throttle_cumulated>>4;

			ui8_adc_regular_flag=0;
		}

		//PAS signal processing
		if(ui8_PAS_flag){

			if(uint32_PAS_counter>100){ //debounce
				uint32_PAS_cumulated -= uint32_PAS_cumulated>>2;
				uint32_PAS_cumulated += uint32_PAS_counter;
				uint32_PAS = uint32_PAS_cumulated>>2;

				uint32_PAS_HIGH_accumulated-=uint32_PAS_HIGH_accumulated>>2;
				uint32_PAS_HIGH_accumulated+=uint32_PAS_HIGH_counter;

				uint32_PAS_fraction=(uint32_PAS_HIGH_accumulated>>2)*100/uint32_PAS;
				uint32_PAS_HIGH_counter=0;
				uint32_PAS_counter =0;
				ui8_PAS_flag=0;
				//read in and sum up torque-signal within one crank revolution (for sempu sensor 32 PAS pulses/revolution, 2^5=32)
				uint32_torque_cumulated -= uint32_torque_cumulated>>5;
#ifdef NCTE
				if(ui16_throttle<ui16_throttle_offset)uint32_torque_cumulated += (ui16_throttle_offset-ui16_throttle);
#else
				if(ui16_throttle>ui16_throttle_offset)uint32_torque_cumulated += (ui16_throttle-ui16_throttle_offset);
#endif


			}
		}

		//SPEED signal processing
#if (SPEEDSOURCE == INTERNAL)
		MS.Speed = uint32_tics_filtered>>3;
#else

		if(ui8_SPEED_flag){
			// HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			if(uint32_SPEED_counter>200){ //debounce
				MS.Speed = uint32_SPEED_counter;
				//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				uint32_SPEED_counter =0;
				ui8_SPEED_flag=0;
				uint32_SPEEDx100_cumulated -=uint32_SPEEDx100_cumulated>>SPEEDFILTER;
				uint32_SPEEDx100_cumulated +=external_tics_to_speedx100(MS.Speed);
			}
		}
#endif
		if(ui8_SPEED_control_flag){
#if (SPEEDSOURCE == INTERNAL)
			uint32_SPEEDx100_cumulated -=uint32_SPEEDx100_cumulated>>SPEEDFILTER;
			uint32_SPEEDx100_cumulated +=internal_tics_to_speedx100(uint32_tics_filtered>>3);
#endif
			ui16_erps=500000/((uint32_tics_filtered>>3)*6);
			ui8_SPEED_control_flag=0;
		}

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG && defined(FAST_LOOP_LOG))
		if(ui8_debug_state==3 && ui8_UART_TxCplt_flag){
			sprintf_(buffer, "%d, %d, %d, %d, %d, %d\r\n", e_log[k][0], e_log[k][1], e_log[k][2],e_log[k][3],e_log[k][4],e_log[k][5]); //>>24
			i=0;
			while (buffer[i] != '\0')
			{i++;}
			ui8_UART_TxCplt_flag=0;
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&buffer, i);
			k++;
			if (k>299){
				k=0;
				ui8_debug_state=0;
				//Obs_flag=0;
			}
		}
#endif

		//--------------------------------------------------------------------------------------------------------------------------------------------------

		//current target calculation
		//highest priority: regen by brake lever


#ifdef ADC_BRAKE
		uint16_mapped_BRAKE = map(ui16_brake_adc, ui16_throttle_offset , THROTTLE_MAX, 0, REGEN_CURRENT);


		if(uint16_mapped_BRAKE>0) brake_flag=1;
		else brake_flag=0;


		if(brake_flag){

			//if(!HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)){
			//if(tics_to_speed(uint32_tics_filtered>>3)>6)int32_current_target=-REGEN_CURRENT; //only apply regen, if motor is turning fast enough
			if(tics_to_speed(uint32_tics_filtered>>3)>6)int32_temp_current_target=uint16_mapped_BRAKE;
			else int32_temp_current_target=0;


#else
			if(HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)) brake_flag=0;
			else brake_flag=1;
			if(brake_flag){

				if(tics_to_speed(uint32_tics_filtered>>3)>6){
					int32_temp_current_target=REGEN_CURRENT; //only apply regen, if motor is turning fast enough
				}
				else int32_temp_current_target=0;

#endif
				int32_temp_current_target= -map(MS.Voltage*CAL_V,BATTERYVOLTAGE_MAX-1000,BATTERYVOLTAGE_MAX,int32_temp_current_target,0);
			}

			//next priority: undervoltage protection
			else if(MS.Voltage<VOLTAGE_MIN)int32_temp_current_target=0;
			//next priority: push assist
			else if(ui8_Push_Assist_flag)int32_temp_current_target=(MS.assist_level*PUSHASSIST_CURRENT)>>8; //does not work for BAFANG and Kunteng protocol actually
			// last priority normal ride conditiones
			else {

#ifdef TS_MODE //torque-sensor mode
				//calculate current target form torque, cadence and assist level
				int32_temp_current_target = (TS_COEF*(int32_t)(MS.assist_level)* (uint32_torque_cumulated>>5)/uint32_PAS)>>8; //>>5 aus Mittelung über eine Kurbelumdrehung, >>8 aus KM5S-Protokoll Assistlevel 0..255

				//limit currest target to max value
				if(int32_temp_current_target>PH_CURRENT_MAX) int32_temp_current_target = PH_CURRENT_MAX;
				//set target to zero, if pedals are not turning
				if(uint32_PAS_counter > PAS_TIMEOUT){
					int32_temp_current_target = 0;
					if(uint32_torque_cumulated>0)uint32_torque_cumulated--; //ramp down cumulated torque value
				}



#else		// torque-simulation mode with throttle override

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
				uint16_mapped_PAS = map(uint32_PAS, RAMP_END, PAS_TIMEOUT, (PH_CURRENT_MAX*(int32_t)(assist_factor[MS.assist_level]))>>8, 0); // level in range 0...5
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
				uint16_mapped_PAS = map(uint32_PAS, RAMP_END, PAS_TIMEOUT, (PH_CURRENT_MAX*(int32_t)(MS.assist_level))/5, 0); // level in range 0...5
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
				uint16_mapped_PAS = map(uint32_PAS, RAMP_END, PAS_TIMEOUT, (PH_CURRENT_MAX*(int32_t)(MS.assist_level-1))>>2, 0); // level in range 1...5
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U||DISPLAY_TYPE == DISPLAY_TYPE_NO2)
				uint16_mapped_PAS = map(uint32_PAS, RAMP_END, PAS_TIMEOUT, ((PH_CURRENT_MAX*(int32_t)(MS.assist_level)))>>8, 0); // level in range 0...255
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
				uint16_mapped_PAS = map(uint32_PAS, RAMP_END, PAS_TIMEOUT, PH_CURRENT_MAX, 0); // Full amps in debug mode
#endif



#ifdef DIRDET
				if (uint32_PAS_counter< PAS_TIMEOUT){
					if ((uint32_PAS_fraction < FRAC_LOW ||uint32_PAS_fraction > FRAC_HIGH)){
						uint16_mapped_PAS= 0;//pedals are turning backwards, stop motor
					}
				}
				else uint32_PAS_HIGH_accumulated=uint32_PAS_cumulated;
#endif //end direction detection


#endif //end if else TQ sensor mode

#ifdef INDIVIDUAL_MODES

				uint16_mapped_PAS = (uint16_mapped_PAS * ui8_speedfactor)>>8;

#endif

#ifdef THROTTLE_OVERRIDE


#ifdef NCTE
				// read in throttle for throttle override
				uint16_mapped_throttle = map(ui16_throttle, THROTTLE_MAX, ui16_throttle_offset,PH_CURRENT_MAX,0);


#else //else NTCE
				// read in throttle for throttle override
				uint16_mapped_throttle = map(ui16_throttle, ui16_throttle_offset, THROTTLE_MAX, 0,PH_CURRENT_MAX);

#endif //end NTCE

#ifndef TS_MODE //normal PAS Mode

				if (uint32_PAS_counter < PAS_TIMEOUT) int32_temp_current_target = uint16_mapped_PAS;		//set current target in torque-simulation-mode, if pedals are turning
				else  {
					int32_temp_current_target= 0;//pedals are not turning, stop motor
					uint32_PAS_cumulated=32000;
					uint32_PAS=32000;
				}

#endif		// end #ifndef TS_MODE
				//check for throttle override
				if(uint16_mapped_throttle>int32_temp_current_target){

#ifdef SPEEDTHROTTLE


					uint16_mapped_throttle = uint16_mapped_throttle*SPEEDLIMIT/PH_CURRENT_MAX;//throttle override: calulate speed target from thottle




					PI_speed.setpoint = uint16_mapped_throttle*100;
					PI_speed.recent_value = internal_tics_to_speedx100(uint32_tics_filtered>>3);
					if( PI_speed.setpoint)SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
					if (internal_tics_to_speedx100(uint32_tics_filtered>>3)<300){//control current slower than 3 km/h
						PI_speed.limit_i=100;
						PI_speed.limit_output=100;
						int32_temp_current_target = PI_control(&PI_speed);

						if(int32_temp_current_target>100)int32_temp_current_target=100;
						if(int32_temp_current_target*i8_direction*i8_reverse_flag<0){
							int32_temp_current_target=0;
						}

					}
					else{


						if(ui8_SPEED_control_flag){//update current target only, if new hall event was detected
							PI_speed.limit_i=PH_CURRENT_MAX;
							PI_speed.limit_output=PH_CURRENT_MAX;
							int32_temp_current_target = PI_control(&PI_speed);
							ui8_SPEED_control_flag=0;
						}
						if(int32_temp_current_target*i8_direction*i8_reverse_flag<0)int32_temp_current_target=0;

					}



#else // else speedthrottle
					int32_temp_current_target=uint16_mapped_throttle;
#endif  //end speedthrottle

				} //end else of throttle override

#endif //end throttle override

			} //end else for normal riding
			//ramp down setpoint at speed limit
#ifdef LEGALFLAG
			if(!brake_flag){ //only ramp down if no regen active
				if(uint32_PAS_counter<PAS_TIMEOUT){
					int32_temp_current_target=map(uint32_SPEEDx100_cumulated>>SPEEDFILTER, MP.speedLimit*100,(MP.speedLimit+2)*100,int32_temp_current_target,0);
				}
				else{ //limit to 6km/h if pedals are not turning
					int32_temp_current_target=map(uint32_SPEEDx100_cumulated>>SPEEDFILTER, 500,700,int32_temp_current_target,0);
				}
			}
			//			else int32_temp_current_target=int32_temp_current_target;

#endif //legalflag

#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER || DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
			if(KM.DirectSetpoint!=-1)int32_temp_current_target=(KM.DirectSetpoint*PH_CURRENT_MAX)>>7;
#endif
			MS.i_q_setpoint=map(MS.Temperature, 120,130,int32_temp_current_target,0); //ramp down power with temperature to avoid overheating the motor
#if(INT_TEMP_25)
			MS.i_q_setpoint=map(MS.int_Temperature, 70,80,MS.i_q_setpoint,0); //ramp down power with processor temperatur to avoid overheating the controller
#endif

			//auto KV detect
			if(ui8_KV_detect_flag){
				MS.i_q_setpoint=ui8_KV_detect_flag;
				if(ui16_KV_detect_counter>32){
					ui8_KV_detect_flag++;
					ui16_KV_detect_counter=0;
					if(MS.u_abs>1900){
						ui8_KV_detect_flag=0;
						HAL_FLASH_Unlock();
						EE_WriteVariable(EEPROM_POS_KV, (int16_t) ui32_KV);
						HAL_FLASH_Lock();
					}
				}
				ui32_KV -=ui32_KV>>4;
				ui32_KV += ((uint32_SPEEDx100_cumulated*_T))/(MS.Voltage*MS.u_q);


			}//end KV detect



			//------------------------------------------------------------------------------------------------------------
			//enable PWM if power is wanted
			if (MS.i_q_setpoint&&!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){

				uint16_half_rotation_counter=0;
				uint16_full_rotation_counter=0;
				TIM1->CCR1 = 1023; //set initial PWM values
				TIM1->CCR2 = 1023;
				TIM1->CCR3 = 1023;
				SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
				if(SystemState == Stop)speed_PLL(0,0,0);//reset integral part
				else {
					PI_iq.integral_part = ((((uint32_SPEEDx100_cumulated*_T))/(MS.Voltage*ui32_KV))<<4)<<PI_iq.shift;
					PI_iq.out=PI_iq.integral_part;
				}
				__HAL_TIM_SET_COUNTER(&htim2,0); //reset tim2 counter
				ui16_timertics=20000; //set interval between two hallevents to a large value
				i8_recent_rotor_direction=i8_direction*i8_reverse_flag*sign(MS.i_q_setpoint);
				get_standstill_position();
			}


			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//slow loop procedere @16Hz, for LEV standard every 4th loop run, send page,
			if(ui32_tim3_counter>500){
				if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				else HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);



				if(ui8_KV_detect_flag){ui16_KV_detect_counter++;}
#if (R_TEMP_PULLUP)
				MS.Temperature = T_NTC(adcData[6]); //Thank you Hendrik ;-)
#else
				MS.Temperature=25;
#endif
				//filter internal temperature reading
				ui32_int_Temp_cumulated-=ui32_int_Temp_cumulated>>5;
				ui32_int_Temp_cumulated+=adcData[7];
				MS.int_Temperature=(((i16_int_Temp_V25-(ui32_int_Temp_cumulated>>5))*24)>>7)+25;

				MS.Voltage=adcData[0];
				if(uint32_SPEED_counter>32000){
					MS.Speed = 32000;
#if (SPEEDSOURCE == EXTERNAL)
					uint32_SPEEDx100_cumulated=0;
#endif
				}

#ifdef INDIVIDUAL_MODES
				// GET recent speedcase for assist profile
				if (uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][1]))ui8_speedcase=0;
				else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][1]) && uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][2]))ui8_speedcase=1;
				else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][2]) && uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][3]))ui8_speedcase=2;
				else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][3]) && uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][4]))ui8_speedcase=3;
				else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][4]))ui8_speedcase=4;

				ui8_speedfactor = map(uint32_tics_filtered>>3,speed_to_tics(assist_profile[0][ui8_speedcase+1]),speed_to_tics(assist_profile[0][ui8_speedcase]),assist_profile[1][ui8_speedcase+1],assist_profile[1][ui8_speedcase]);


#endif
				//check if rotor is turning

				if((uint16_full_rotation_counter>7999||uint16_half_rotation_counter>7999)){
					SystemState = Stop;
					//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					if(READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){
						CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning
						uint32_tics_filtered=1000000;
						get_standstill_position();
					}

				}
				else if(ui8_6step_flag) SystemState = SixStep;
				else SystemState = Running;

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG && !defined(FAST_LOOP_LOG))
				//print values for debugging

				sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d\r\n",
						adcData[1],
						ui16_throttle_offset,
						ui16_timertics,
						uint32_PAS,
						MS.Battery_Current,
						int32_temp_current_target ,
						MS.i_q,
						MS.u_abs,
						SystemState);
				// sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d\r\n",(uint16_t)adcData[0],(uint16_t)adcData[1],(uint16_t)adcData[2],(uint16_t)adcData[3],(uint16_t)(adcData[4]),(uint16_t)(adcData[5]),(uint16_t)(adcData[6])) ;
				// sprintf_(buffer, "%d, %d, %d, %d, %d, %d\r\n",tic_array[0],tic_array[1],tic_array[2],tic_array[3],tic_array[4],tic_array[5]) ;
				i=0;
				while (buffer[i] != '\0')
				{i++;}
				HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&buffer, i);


				ui8_print_flag=0;

#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_EBiCS)
				ui8_slowloop_counter++;
				if(ui8_slowloop_counter>3){
					ui8_slowloop_counter = 0;

					switch (ui8_main_LEV_Page_counter){
					case 1: {
						ui8_LEV_Page_to_send = 1;
					}
					break;
					case 2: {
						ui8_LEV_Page_to_send = 2;
					}
					break;
					case 3: {
						ui8_LEV_Page_to_send = 3;
					}
					break;
					case 4: {
						//to do, define other pages
						ui8_LEV_Page_to_send = 4;
					}
					break;
					}//end switch

					//  send_ant_page(ui8_LEV_Page_to_send, &MS, &MP);

					ui8_main_LEV_Page_counter++;
					if(ui8_main_LEV_Page_counter>4)ui8_main_LEV_Page_counter=1;
				}

#endif
				ui32_tim3_counter=0;
			}// end of slow loop

			/* USER CODE END WHILE */

			/* USER CODE BEGIN 3 */

		}
		/* USER CODE END 3 */

	}

	/**
	 * @brief System Clock Configuration
	 * @retval None
	 */
	void SystemClock_Config(void)
	{

		RCC_OscInitTypeDef RCC_OscInitStruct;
		RCC_ClkInitTypeDef RCC_ClkInitStruct;
		RCC_PeriphCLKInitTypeDef PeriphClkInit;

		/**Initializes the CPU, AHB and APB busses clocks
		 */
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		RCC_OscInitStruct.HSICalibrationValue = 16;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Initializes the CPU, AHB and APB busses clocks
		 */
		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
				|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
		PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure the Systick interrupt time
		 */
		HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

		/**Configure the Systick
		 */
		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

		/* SysTick_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	}


	/* ADC1 init function */
	static void MX_ADC1_Init(void)
	{

		ADC_MultiModeTypeDef multimode;
		ADC_InjectionConfTypeDef sConfigInjected;
		ADC_ChannelConfTypeDef sConfig;

		/**Common config
		 */
		hadc1.Instance = ADC1;
		hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; //Scan muÃ fÃ¼r getriggerte Wandlung gesetzt sein
		hadc1.Init.ContinuousConvMode = DISABLE;
		hadc1.Init.DiscontinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;// Trigger regular ADC with timer 3 ADC_EXTERNALTRIGCONV_T1_CC1;// // ADC_SOFTWARE_START; //
		hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc1.Init.NbrOfConversion = 8;
		hadc1.Init.NbrOfDiscConversion = 0;


		if (HAL_ADC_Init(&hadc1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure the ADC multi-mode
		 */
		multimode.Mode = ADC_DUALMODE_REGSIMULT_INJECSIMULT;
		if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Injected Channel
		 */
		sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
		sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
		sConfigInjected.InjectedNbrOfConversion = 1;
		sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4; // Hier bin ich nicht sicher ob Trigger out oder direkt CC4
		sConfigInjected.AutoInjectedConv = DISABLE; //muÃ aus sein
		sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
		sConfigInjected.InjectedOffset = ui16_ph1_offset;//1900;
		HAL_ADC_Stop(&hadc1); //ADC muÃ gestoppt sein, damit Triggerquelle gesetzt werden kann.
		if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_7; //battery voltage
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}


		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_3; //Connector SP: throttle input
		sConfig.Rank = ADC_REGULAR_RANK_2;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_4; //Phase current 1
		sConfig.Rank = ADC_REGULAR_RANK_3;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_5; //Phase current 2
		sConfig.Rank = ADC_REGULAR_RANK_4;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_6; //Phase current 3
		sConfig.Rank = ADC_REGULAR_RANK_5;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = ADC_REGULAR_RANK_6; // connector AD2
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_9; // connector AD1, temperature or torque input for Controller from PhoebeLiu @ aliexpress
		sConfig.Rank = ADC_REGULAR_RANK_7;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_TEMPSENSOR; // internal STM32 temperature sensor
		sConfig.Rank = ADC_REGULAR_RANK_8;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

	}

	/* ADC2 init function */
	static void MX_ADC2_Init(void)
	{

		ADC_InjectionConfTypeDef sConfigInjected;

		/**Common config
		 */
		hadc2.Instance = ADC2;
		hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE; //hier auch Scan enable?!
		hadc2.Init.ContinuousConvMode = DISABLE;
		hadc2.Init.DiscontinuousConvMode = DISABLE;
		hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc2.Init.NbrOfConversion = 1;
		if (HAL_ADC_Init(&hadc2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Injected Channel
		 */
		sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
		sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
		sConfigInjected.InjectedNbrOfConversion = 1;
		sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
		sConfigInjected.AutoInjectedConv = DISABLE;
		sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
		sConfigInjected.InjectedOffset = ui16_ph2_offset;//	1860;
		if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

	}
	/* TIM1 init function */
	static void MX_TIM1_Init(void)
	{

		TIM_ClockConfigTypeDef sClockSourceConfig;
		TIM_MasterConfigTypeDef sMasterConfig;
		TIM_OC_InitTypeDef sConfigOC;
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

		htim1.Instance = TIM1;
		htim1.Init.Prescaler = 0;
		htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
		htim1.Init.Period = _T;
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.RepetitionCounter = 0;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 1;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		//sConfigOC.OCMode = TIM_OCMODE_ACTIVE; // war hier ein Bock?!
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
		if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		sBreakDeadTimeConfig.DeadTime = 32;
		sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		HAL_TIM_MspPostInit(&htim1);

	}

	/* TIM2 init function */
	static void MX_TIM2_Init(void)
	{
		TIM_ClockConfigTypeDef sClockSourceConfig;
		TIM_SlaveConfigTypeDef sSlaveConfig;
		TIM_MasterConfigTypeDef sMasterConfig;
		TIM_IC_InitTypeDef sConfigIC;

		htim2.Instance = TIM2;
		htim2.Init.Prescaler = 128;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = 65535;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
		sSlaveConfig.InputTrigger = TIM_TS_TI1F_ED;
		sSlaveConfig.TriggerFilter = 8;
		if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 15;
		if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_ConfigTI1Input(&htim2, TIM_TI1SELECTION_XORCOMBINATION) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

	}

	/* TIM3 init function 8kHz interrupt frequency for regular adc triggering */
	static void MX_TIM3_Init(void)
	{

		TIM_ClockConfigTypeDef sClockSourceConfig;
		TIM_MasterConfigTypeDef sMasterConfig;

		htim3.Instance = TIM3;
		htim3.Init.Prescaler = 0;
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = 7813;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		HAL_TIM_MspPostInit(&htim3);

	}

	/* USART1 init function */
	static void MX_USART1_UART_Init(void)
	{

		huart1.Instance = USART1;

#if ((DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER) ||DISPLAY_TYPE==DISPLAY_TYPE_KUNTENG||DISPLAY_TYPE==DISPLAY_TYPE_EBiCS||DISPLAY_TYPE==DISPLAY_TYPE_NO2)
		huart1.Init.BaudRate = 9600;
#elif (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
		huart1.Init.BaudRate = 1200;
#else
		huart1.Init.BaudRate = 56000;
#endif


		huart1.Init.WordLength = UART_WORDLENGTH_8B;
		huart1.Init.StopBits = UART_STOPBITS_1;
		huart1.Init.Parity = UART_PARITY_NONE;
		huart1.Init.Mode = UART_MODE_TX_RX;
		huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart1.Init.OverSampling = UART_OVERSAMPLING_16;
		if (HAL_UART_Init(&huart1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	}

	/**
	 * Enable DMA controller clock
	 */
	static void MX_DMA_Init(void)
	{
		/* DMA controller clock enable */
		__HAL_RCC_DMA1_CLK_ENABLE();



		/* DMA interrupt init */
		/* DMA1_Channel1_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
		//  /* DMA1_Channel4_IRQn interrupt configuration */
		//  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 3, 0);
		//  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
		//  /* DMA1_Channel5_IRQn interrupt configuration */
		//  //HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
		//  //HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	}

	/** Configure pins as
	 * Analog
	 * Input
	 * Output
	 * EVENT_OUT
	 * EXTI
	 */
	static void MX_GPIO_Init(void)
	{

		GPIO_InitTypeDef GPIO_InitStruct;

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		/*Configure GPIO pins : Hall_1_Pin Hall_2_Pin Hall_3_Pin */
		GPIO_InitStruct.Pin = Hall_1_Pin|Hall_2_Pin|Hall_3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*Configure GPIO pin : LED_Pin */
		GPIO_InitStruct.Pin = LED_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : LIGHT_Pin */
		GPIO_InitStruct.Pin = LIGHT_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(LIGHT_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : BRAKE_LIGHT_Pin */
		GPIO_InitStruct.Pin = BRAKE_LIGHT_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(BRAKE_LIGHT_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : Brake_Pin */
		GPIO_InitStruct.Pin = Brake_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(Brake_GPIO_Port, &GPIO_InitStruct);


		/*Configure GPIO pins : Speed_EXTI5_Pin PAS_EXTI8_Pin */
		GPIO_InitStruct.Pin = Speed_EXTI5_Pin|PAS_EXTI8_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


		/* EXTI interrupt init*/


		HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	}

	/* USER CODE BEGIN 4 */

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		if (htim == &htim3) {

#if SPEED_PLL
			if(!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE))q31_rotorposition_PLL += (q31_angle_per_tic<<1);
#endif

			if(ui32_tim3_counter<32000)ui32_tim3_counter++;
			if (uint32_PAS_counter < PAS_TIMEOUT+1){
				uint32_PAS_counter++;
				if(HAL_GPIO_ReadPin(PAS_GPIO_Port, PAS_Pin))uint32_PAS_HIGH_counter++;
			}
			if (uint32_SPEED_counter<128000)uint32_SPEED_counter++;					//counter for external Speedsensor
			if(uint16_full_rotation_counter<8000)uint16_full_rotation_counter++;	//full rotation counter for motor standstill detection
			if(uint16_half_rotation_counter<8000)uint16_half_rotation_counter++;	//half rotation counter for motor standstill detection

		}
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}



	// regular ADC callback
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	{
		ui8_adc_regular_flag=1;

	}

	//injected ADC

	void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
	{
		//for oszi-check of used time in FOC procedere
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		ui32_tim1_counter++;

		/*  else {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  uint32_SPEED_counter=0;
	  }*/

		if(!ui8_adc_offset_done_flag)
		{
			i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
			i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

			ui8_adc_inj_flag=1;
		}
		else{

#ifdef DISABLE_DYNAMIC_ADC

			i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
			i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);


#else
			switch (MS.char_dyn_adc_state) //read in according to state
			{
			case 1: //Phase C at high dutycycles, read from A+B directly
			{
				temp1=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				i16_ph1_current = temp1 ;

				temp2=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
				i16_ph2_current = temp2;
			}
			break;
			case 2: //Phase A at high dutycycles, read from B+C (A = -B -C)
			{

				temp2=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
				i16_ph2_current = temp2;

				temp1=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				i16_ph1_current = -i16_ph2_current-temp1;

			}
			break;
			case 3: //Phase B at high dutycycles, read from A+C (B=-A-C)
			{
				temp1=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				i16_ph1_current = temp1 ;
				temp2=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
				i16_ph2_current = -i16_ph1_current-temp2;
			}
			break;

			case 0: //timeslot too small for ADC
			{
				//do nothing
			}
			break;




			} // end case
#endif


			__disable_irq(); //ENTER CRITICAL SECTION!!!!!!!!!!!!!

			//extrapolate recent rotor position
			ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last event
			if (MS.hall_angle_detect_flag) {
				if(ui16_timertics<SIXSTEPTHRESHOLD && ui16_tim2_recent<200)ui8_6step_flag=0;
				if(ui16_timertics>(SIXSTEPTHRESHOLD*6)>>2)ui8_6step_flag=1;


				if(MS.angle_est){
					q31_rotorposition_PLL += q31_angle_per_tic;
				}
				if (ui16_tim2_recent < ui16_timertics+(ui16_timertics>>2) && !ui8_overflow_flag && !ui8_6step_flag) { //prevent angle running away at standstill
					if(MS.angle_est&&iabs(q31_PLL_error)<deg_30){
						q31_rotorposition_absolute=q31_rotorposition_PLL;
						MS.system_state=PLL;
					}
					else{
						q31_rotorposition_absolute = q31_rotorposition_hall
								+ (q31_t) (i8_recent_rotor_direction
										* ((10923 * ui16_tim2_recent) / ui16_timertics)
										<< 16); //interpolate angle between two hallevents by scaling timer2 tics, 10923<<16 is 715827883 = 60deg
						MS.system_state=Interpolation;
					}
				} else {
					ui8_overflow_flag = 1;
					if(MS.KV_detect_flag)q31_rotorposition_absolute = q31_rotorposition_hall;
					else q31_rotorposition_absolute = q31_rotorposition_hall+i8_direction*sign(MS.i_q_setpoint)*deg_30;//offset of 30 degree to get the middle of the sector
					MS.system_state=SixStep;
					//	}

				}
			} //end if hall angle detect
			//temp2=(((q31_rotorposition_absolute >> 23) * 180) >> 8);
			__enable_irq(); //EXIT CRITICAL SECTION!!!!!!!!!!!!!!

#ifndef DISABLE_DYNAMIC_ADC

			//get the Phase with highest duty cycle for dynamic phase current reading
			dyn_adc_state(q31_rotorposition_absolute);
			//set the according injected channels to read current at Low-Side active time

			if (MS.char_dyn_adc_state!=char_dyn_adc_state_old){
				set_inj_channel(MS.char_dyn_adc_state);
				char_dyn_adc_state_old = MS.char_dyn_adc_state;
			}
#endif

			//int16_current_target=0;
			// call FOC procedure if PWM is enabled

			if (READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){
				FOC_calculation(i16_ph1_current, i16_ph2_current, q31_rotorposition_absolute, (((int16_t)i8_direction*i8_reverse_flag)*MS.i_q_setpoint), &MS);
			}
			//temp5=__HAL_TIM_GET_COUNTER(&htim1);
			//set PWM

			TIM1->CCR1 =  (uint16_t) switchtime[0];
			TIM1->CCR2 =  (uint16_t) switchtime[1];
			TIM1->CCR3 =  (uint16_t) switchtime[2];
			//__enable_irq();


			//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		} // end else

	}

	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
	{
		//__HAL_TIM_SET_COUNTER(&htim2,0); //reset tim2 counter
		//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

		temp5=TIM3->CNT;

		if(TIM2->CCR1>20)ui16_timertics = TIM2->CCR1; //debounce hall signals


		//Hall sensor event processing

		ui8_hall_state = GPIOA->IDR & 0b111; //Mask input register with Hall 1 - 3 bits


		ui8_hall_case=ui8_hall_state_old*10+ui8_hall_state;
		if(MS.hall_angle_detect_flag){ //only process, if autodetect procedere is fininshed
			ui8_hall_state_old=ui8_hall_state;
		}

		uint32_tics_filtered-=uint32_tics_filtered>>3;
		uint32_tics_filtered+=ui16_timertics;

		ui8_overflow_flag=0;
		ui8_SPEED_control_flag=1;



		switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
		{
		//6 cases for forward direction
		//6 cases for forward direction
		case 64:
			q31_rotorposition_hall = Hall_64;

			i8_recent_rotor_direction = -i16_hall_order;
			uint16_full_rotation_counter = 0;
			break;
		case 45:
			q31_rotorposition_hall = Hall_45;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 51:
			q31_rotorposition_hall = Hall_51;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 13:
			q31_rotorposition_hall = Hall_13;

			i8_recent_rotor_direction = -i16_hall_order;
			uint16_half_rotation_counter = 0;
			break;
		case 32:
			q31_rotorposition_hall = Hall_32;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 26:
			q31_rotorposition_hall = Hall_26;

			i8_recent_rotor_direction = -i16_hall_order;
			break;

			//6 cases for reverse direction
		case 46:
			q31_rotorposition_hall = Hall_64;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 62:
			q31_rotorposition_hall = Hall_26;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 23:
			q31_rotorposition_hall = Hall_32;

			i8_recent_rotor_direction = i16_hall_order;
			uint16_half_rotation_counter = 0;
			break;
		case 31:
			q31_rotorposition_hall = Hall_13;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 15:
			q31_rotorposition_hall = Hall_51;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 54:
			q31_rotorposition_hall = Hall_45;

			i8_recent_rotor_direction = i16_hall_order;
			uint16_full_rotation_counter = 0;
			break;

		} // end case

		if(MS.angle_est){
			q31_PLL_error=q31_rotorposition_PLL-q31_rotorposition_hall;
			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall,0);
		}

#if SPEED_PLL
		if(ui16_erps>30){   //360 interpolation at higher erps
			if(ui8_hall_case==32||ui8_hall_case==23){
				q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));

			}
		}
		else{

			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));
		}

#endif

		//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


	}

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		//PAS processing
		if(GPIO_Pin == PAS_EXTI8_Pin)
		{
			ui8_PAS_flag = 1;
		}

		//Speed processing
		if(GPIO_Pin == Speed_EXTI5_Pin)
		{

			ui8_SPEED_flag = 1; //with debounce

		}
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
	{
		//ui8_UART_flag=1;

	}

	void UART_IdleItCallback(void)
	{
		ui8_UART_flag=1;

	}

	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
	{
		ui8_UART_TxCplt_flag=1;
	}

	void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
		KingMeter_Init (&KM);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
		Bafang_Init (&BF);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
		kunteng_init();
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_EBiCS)
		//       ebics_init();
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
	No2_Init(&No2);
#endif
	}

	void get_internal_temp_offset(void){
		int32_t temp=0;
		for(i=0;i<32;i++){
			while(!ui8_adc_regular_flag){}
			temp+=adcData[7];
			ui8_adc_regular_flag=0;
		}
		HAL_FLASH_Unlock();
		EE_WriteVariable(EEPROM_INT_TEMP_V25,temp>>5);
		HAL_FLASH_Lock();
	}
#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
	void No2_update(void)
	{
		/* Prepare Tx parameters */

#if (SPEEDSOURCE  == EXTERNAL)
		No2.Tx.Wheeltime_ms = ((MS.Speed>>3)*PULSES_PER_REVOLUTION); //>>3 because of 8 kHz counter frequency, so 8 tics per ms
#else
		if(__HAL_TIM_GET_COUNTER(&htim2) < 12000)
		{
			No2.Tx.Wheeltime_ms = (MS.Speed*GEAR_RATIO*6)>>9; //>>9 because of 500kHZ timer2 frequency, 512 tics per ms should be OK *6 because of 6 hall interrupts per electric revolution.

		}
		else
		{
			No2.Tx.Wheeltime_ms = 64000;
		}

#endif
		if(MS.Temperature>130) No2.Tx.Error = 7;  //motor failure
		else if(MS.int_Temperature>80)No2.Tx.Error = 9; //controller failure
		else No2.Tx.Error = 0; //no failure


		No2.Tx.Current_x10 = (uint16_t) (MS.Battery_Current/100); //MS.Battery_Current is in mA
		No2.Tx.BrakeActive=brake_flag;

		/* Apply Rx parameters */

		MS.assist_level = No2.Rx.AssistLevel;

		if(!No2.Rx.Headlight)
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);

		}
		else // KM_HEADLIGHT_ON, KM_HEADLIGHT_LOW, KM_HEADLIGHT_HIGH
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);

		}


		if(No2.Rx.PushAssist)
		{
			ui8_Push_Assist_flag=1;
		}
		else
		{
			ui8_Push_Assist_flag=0;
		}

	}

#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER || DISPLAY_TYPE & DISPLAY_TYPE_DEBUG)
	void kingmeter_update(void)
	{
		/* Prepare Tx parameters */

		if(battery_percent_fromcapacity > 10)
		{
			KM.Tx.Battery = KM_BATTERY_NORMAL;
		}
		else
		{
			KM.Tx.Battery = KM_BATTERY_LOW;
		}


#if (SPEEDSOURCE  == EXTERNAL)
		KM.Tx.Wheeltime_ms = ((MS.Speed>>3)*PULSES_PER_REVOLUTION); //>>3 because of 8 kHz counter frequency, so 8 tics per ms
#else
		if(__HAL_TIM_GET_COUNTER(&htim2) < 12000)
		{
			KM.Tx.Wheeltime_ms = (MS.Speed*GEAR_RATIO*6)>>9; //>>9 because of 500kHZ timer2 frequency, 512 tics per ms should be OK *6 because of 6 hall interrupts per electric revolution.

		}
		else
		{
			KM.Tx.Wheeltime_ms = 64000;
		}

#endif
		if(MS.Temperature>130) KM.Tx.Error = KM_ERROR_OVHT;
		else if(MS.int_Temperature>80)KM.Tx.Error = KM_ERROR_IOVHT;
		else KM.Tx.Error = KM_ERROR_NONE;


		KM.Tx.Current_x10 = (uint16_t) (MS.Battery_Current/100); //MS.Battery_Current is in mA


		/* Receive Rx parameters/settings and send Tx parameters */
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
		KingMeter_Service(&KM);
#endif


		/* Apply Rx parameters */

		MS.assist_level = KM.Rx.AssistLevel;

		if(KM.Rx.Headlight == KM_HEADLIGHT_OFF)
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);

		}
		else // KM_HEADLIGHT_ON, KM_HEADLIGHT_LOW, KM_HEADLIGHT_HIGH
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);

		}


		if(KM.Rx.PushAssist == KM_PUSHASSIST_ON)
		{
			ui8_Push_Assist_flag=1;
		}
		else
		{
			ui8_Push_Assist_flag=0;
		}
//	    if( KM.Settings.Reverse)i8_direction = -1;
//	    else i8_direction = 1;
		//    MP.speedLimit=KM.Rx.SPEEDMAX_Limit;
		//    MP.battery_current_max = KM.Rx.CUR_Limit_mA;



	}

#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
	void bafang_update(void)
	{
		/* Prepare Tx parameters */

		if(MS.Voltage*CAL_BAT_V>BATTERY_LEVEL_5)battery_percent_fromcapacity=95;
		else if(MS.Voltage*CAL_BAT_V>BATTERY_LEVEL_4)battery_percent_fromcapacity=80;
		else if(MS.Voltage*CAL_BAT_V>BATTERY_LEVEL_3)battery_percent_fromcapacity=50;
		else if(MS.Voltage*CAL_BAT_V>BATTERY_LEVEL_2)battery_percent_fromcapacity=30;
		else if(MS.Voltage*CAL_BAT_V>BATTERY_LEVEL_1)battery_percent_fromcapacity=20;
		else battery_percent_fromcapacity=5;


		BF.Tx.Battery = battery_percent_fromcapacity;



#if (SPEEDSOURCE == EXTERNAL)    // Adapt wheeltime to match displayed speedo value according config.h setting

		if(uint32_SPEED_counter<16000) BF.Tx.Speed = (external_tics_to_speedx100(MS.Speed)*20)>>8;// Geschwindigkeit ist Weg pro Zeit Radumfang durch Dauer einer Radumdrehung --> Umfang * 8000*3600/(n*1000000) * Skalierung Bafang Display 200/26,6
		else BF.Tx.Speed = 0;

#else
		if(__HAL_TIM_GET_COUNTER(&htim2) < 12000)
		{
			BF.Tx.Speed =(internal_tics_to_speedx100(MS.Speed)*20)>>8; //factor is *20/256, found empiric

		}
		else
		{
			BF.Tx.Speed = 0;
		}
#endif

		BF.Tx.Power = (MS.Battery_Current/500)&0xFF; // Unit: 1 digit --> 0.5 A, MS.Battery_Current is in mA


		/* Receive Rx parameters/settings and send Tx parameters */
		Bafang_Service(&BF,1);



		/* Apply Rx parameters */

		//No headlight supported on my controller hardware.
		if(BF.Rx.Headlight)
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);

		}
		else
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);

		}


		if(BF.Rx.PushAssist) ui8_Push_Assist_flag=1;
		else ui8_Push_Assist_flag=0;

		MS.assist_level=BF.Rx.AssistLevel;
	}

#endif

	int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
	{
		// if input is smaller/bigger than expected return the min/max out ranges value
		if (x < in_min)
			return out_min;
		else if (x > in_max)
			return out_max;

		// map the input to the output range.
		// round up if mapping bigger ranges to smaller ranges
		else  if ((in_max - in_min) > (out_max - out_min))
			return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
		// round down if mapping smaller ranges to bigger ranges
		else
			return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	//assuming, a proper AD conversion takes 350 timer tics, to be confirmed. DT+TR+TS deadtime + noise subsiding + sample time
	void dyn_adc_state(q31_t angle){
		if (switchtime[2]>switchtime[0] && switchtime[2]>switchtime[1]){
			MS.char_dyn_adc_state = 1; // -90Â° .. +30Â°: Phase C at high dutycycles
			if(switchtime[2]>1500)TIM1->CCR4 =  switchtime[2]-TRIGGER_OFFSET_ADC;
			else TIM1->CCR4 = TRIGGER_DEFAULT;
		}

		if (switchtime[0]>switchtime[1] && switchtime[0]>switchtime[2]) {
			MS.char_dyn_adc_state = 2; // +30Â° .. 150Â° Phase A at high dutycycles
			if(switchtime[0]>1500)TIM1->CCR4 =  switchtime[0]-TRIGGER_OFFSET_ADC;
			else TIM1->CCR4 = TRIGGER_DEFAULT;
		}

		if (switchtime[1]>switchtime[0] && switchtime[1]>switchtime[2]){
			MS.char_dyn_adc_state = 3; // +150 .. -90Â° Phase B at high dutycycles
			if(switchtime[1]>1500)TIM1->CCR4 =  switchtime[1]-TRIGGER_OFFSET_ADC;
			else TIM1->CCR4 = TRIGGER_DEFAULT;
		}
	}

	static void set_inj_channel(char state){
		switch (state)
		{
		case 1: //Phase C at high dutycycles, read current from phase A + B
		{
			ADC1->JSQR=0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
			ADC1->JOFR1 = ui16_ph1_offset;
			ADC2->JSQR=0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
			ADC2->JOFR1 = ui16_ph2_offset;


		}
		break;
		case 2: //Phase A at high dutycycles, read current from phase C + B
		{
			ADC1->JSQR=0b00110000000000000000; //ADC1 injected reads phase C, JSQ4 = 0b00110, decimal 6
			ADC1->JOFR1 = ui16_ph3_offset;
			ADC2->JSQR=0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
			ADC2->JOFR1 = ui16_ph2_offset;


		}
		break;

		case 3: //Phase B at high dutycycles, read current from phase A + C
		{
			ADC1->JSQR=0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
			ADC1->JOFR1 = ui16_ph1_offset;
			ADC2->JSQR=0b00110000000000000000; //ADC2 injected reads phase C, JSQ4 = 0b00110, decimal 6
			ADC2->JOFR1 = ui16_ph3_offset;


		}
		break;


		}


	}
	uint8_t throttle_is_set(void){
		if(uint16_mapped_throttle > 0)
		{
			return 1;
		}
		else return 0;
	}
	void autodetect() {
		SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		MS.hall_angle_detect_flag = 0; //set uq to contstant value in FOC.c for open loop control
		q31_rotorposition_absolute = 1 << 31;
		i16_hall_order = 1;//reset hall order
		MS.i_d_setpoint= 300; //set MS.id to appr. 2000mA
		MS.i_q_setpoint= 0;
		//	uint8_t zerocrossing = 0;
		//	q31_t diffangle = 0;
		HAL_Delay(5);
		for (i = 0; i < 1080; i++) {
			HAL_IWDG_Refresh(&hiwdg);
			q31_rotorposition_absolute += 11930465; //drive motor in open loop with steps of 1 deg
			HAL_Delay(5);
			//printf_("%d, %d, %d, %d\n", temp3>>16,temp4>>16,temp5,temp6);

			if (ui8_hall_state_old != ui8_hall_state) {
				printf_("angle: %d, hallstate:  %d, hallcase %d \n",
						(int16_t) (((q31_rotorposition_absolute >> 23) * 180) >> 8),
						ui8_hall_state, ui8_hall_case);

				switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
				{
				//6 cases for forward direction
				case 64:
					Hall_64=q31_rotorposition_absolute;
					break;
				case 45:
					Hall_45=q31_rotorposition_absolute;
					break;
				case 51:
					Hall_51=q31_rotorposition_absolute;
					break;
				case 13:
					Hall_13=q31_rotorposition_absolute;
					break;
				case 32:
					Hall_32=q31_rotorposition_absolute;
					break;
				case 26:
					Hall_26=q31_rotorposition_absolute;
					break;

					//6 cases for reverse direction
				case 46:
					Hall_64=q31_rotorposition_absolute;
					break;
				case 62:
					Hall_26=q31_rotorposition_absolute;
					break;
				case 23:
					Hall_32=q31_rotorposition_absolute;
					break;
				case 31:
					Hall_13=q31_rotorposition_absolute;
					break;
				case 15:
					Hall_51=q31_rotorposition_absolute;
					break;
				case 54:
					Hall_45=q31_rotorposition_absolute;
					break;

				} // end case


				ui8_hall_state_old = ui8_hall_state;
			}
		}

		CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning
		TIM1->CCR1 = 1023; //set initial PWM values
		TIM1->CCR2 = 1023;
		TIM1->CCR3 = 1023;
		MS.hall_angle_detect_flag=1;
		MS.i_d = 0;
		MS.i_q = 0;
		MS.u_d=0;
		MS.u_q=0;
		MS.i_d_setpoint= 0;
		uint32_tics_filtered=1000000;

		HAL_FLASH_Unlock();

		if (i8_recent_rotor_direction == 1) {
			EE_WriteVariable(EEPROM_POS_HALL_ORDER, 1);
			i16_hall_order = 1;
		} else {
			EE_WriteVariable(EEPROM_POS_HALL_ORDER, -1);
			i16_hall_order = -1;
		}
		EE_WriteVariable(EEPROM_POS_HALL_45, Hall_45 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_51, Hall_51 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_13, Hall_13 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_32, Hall_32 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_26, Hall_26 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_64, Hall_64 >> 16);

		HAL_FLASH_Lock();

		MS.hall_angle_detect_flag = 1;

		HAL_Delay(5);
		ui8_KV_detect_flag = 30;


	}

	void get_standstill_position(){
		HAL_Delay(100);
		HAL_TIM_IC_CaptureCallback(&htim2); //read in initial rotor position

		switch (ui8_hall_state) {
		//6 cases for forward direction
		case 2:
			q31_rotorposition_hall = Hall_32;
			break;
		case 6:
			q31_rotorposition_hall = Hall_26;
			break;
		case 4:
			q31_rotorposition_hall = Hall_64;
			break;
		case 5:
			q31_rotorposition_hall = Hall_45;
			break;
		case 1:
			q31_rotorposition_hall = Hall_51;

			break;
		case 3:
			q31_rotorposition_hall = Hall_13;
			break;

		}

		q31_rotorposition_absolute = q31_rotorposition_hall;
	}

	int32_t speed_to_tics (uint8_t speed){
		return WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*speed*10);
	}

	int8_t tics_to_speed (uint32_t tics){
		return WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*tics*10);
	}

	int16_t internal_tics_to_speedx100 (uint32_t tics){
		return WHEEL_CIRCUMFERENCE*50*3600/(6*GEAR_RATIO*tics);
	}

	int16_t external_tics_to_speedx100 (uint32_t tics){
		return WHEEL_CIRCUMFERENCE*8*360/(PULSES_PER_REVOLUTION*tics);
	}

	void runPIcontrol(){


		q31_t_Battery_Current_accumulated -= q31_t_Battery_Current_accumulated>>8;
		q31_t_Battery_Current_accumulated += ((MS.i_q*MS.u_abs)>>11)*(uint16_t)(CAL_I>>8);

		MS.Battery_Current = (q31_t_Battery_Current_accumulated>>8)*i8_direction*i8_reverse_flag; //Battery current in mA
		//Check battery current limit
		if(MS.Battery_Current>MP.battery_current_max) ui8_BC_limit_flag=1;
		if(MS.Battery_Current<-REGEN_CURRENT_MAX) ui8_BC_limit_flag=1;
		//reset battery current flag with small hysteresis
		if(brake_flag==0){
			//if(HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)){
			if(((MS.i_q_setpoint*MS.u_abs)>>11)*(uint16_t)(CAL_I>>8)<(MP.battery_current_max*7)>>3)ui8_BC_limit_flag=0;
		}
		else{
			if(((MS.i_q_setpoint*MS.u_abs)>>11)*(uint16_t)(CAL_I>>8)>(-REGEN_CURRENT_MAX*7)>>3)ui8_BC_limit_flag=0;
		}

		//control iq

		//if
		if (!ui8_BC_limit_flag){
			PI_iq.recent_value = MS.i_q;
			PI_iq.setpoint = i8_direction*i8_reverse_flag*MS.i_q_setpoint;
		}
		else{
			if(brake_flag==0){
				// if(HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)){
				PI_iq.recent_value=  (MS.Battery_Current>>6)*i8_direction*i8_reverse_flag;
				PI_iq.setpoint = (MP.battery_current_max>>6)*i8_direction*i8_reverse_flag;
			}
			else{
				PI_iq.recent_value=  (MS.Battery_Current>>6)*i8_direction*i8_reverse_flag;
				PI_iq.setpoint = (-REGEN_CURRENT_MAX>>6)*i8_direction*i8_reverse_flag;
			}
		}

		q31_u_q_temp =  PI_control(&PI_iq);

		//Control id
		PI_id.recent_value = MS.i_d;
		PI_id.setpoint = MS.i_d_setpoint;
		q31_u_d_temp = -PI_control(&PI_id); //control direct current to zero


		//limit voltage in rotating frame, refer chapter 4.10.1 of UM1052
		//MS.u_abs = (q31_t)hypot((double)q31_u_d_temp, (double)q31_u_q_temp); //absolute value of U in static frame
		arm_sqrt_q31((q31_u_d_temp*q31_u_d_temp+q31_u_q_temp*q31_u_q_temp)<<1,&MS.u_abs);
		MS.u_abs = (MS.u_abs>>16)+1;


		if (MS.u_abs > _U_MAX){
			MS.u_q = (q31_u_q_temp*_U_MAX)/MS.u_abs; //division!
			MS.u_d = (q31_u_d_temp*_U_MAX)/MS.u_abs; //division!
			MS.u_abs = _U_MAX;
		}
		else{
			MS.u_q=q31_u_q_temp;
			MS.u_d=q31_u_d_temp;
		}

		PI_flag=0;
	}

	q31_t speed_PLL (q31_t ist, q31_t soll, uint8_t speedadapt)
	{
		q31_t q31_p;
		static q31_t q31_d_i = 0;
		static q31_t q31_d_dc = 0;
		// temp6 = soll-ist;
		// temp5 = speedadapt;
		q31_p=(soll - ist)>>(P_FACTOR_PLL-speedadapt);   				//7 for Shengyi middrive, 10 for BionX IGH3
		q31_d_i+=(soll - ist)>>(I_FACTOR_PLL-speedadapt);				//11 for Shengyi middrive, 10 for BionX IGH3

		//clamp i part to twice the theoretical value from hall interrupts
		if (q31_d_i>((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i = ((deg_30>>18)*500/ui16_timertics)<<16;
		if (q31_d_i<-((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i =- ((deg_30>>18)*500/ui16_timertics)<<16;


		if (!ist&&!soll)q31_d_i=0;

		q31_d_dc=q31_p+q31_d_i;
		return (q31_d_dc);
	}

#if (R_TEMP_PULLUP)
	int16_t T_NTC(uint16_t ADC) // ADC 12 Bit, 10k Pullup, Rückgabewert in °C

	{
		uint16_t Ux1000 = 3300;
		uint16_t U2x1000 = ADC*Ux1000/4095;
		uint16_t R1 = R_TEMP_PULLUP;
		uint32_t R = U2x1000*R1/(Ux1000-U2x1000);
		// 	printf("R= %d\r\n",R);
		//  printf("u2= %d\r\n",U2x1000);
		if(R >> 19) return -44;
		uint16_t n = 0;
		while(R >> n > 1) n++;
		R <<= 13;
		for(n <<= 6; R >> (n >> 6) >> 13; n++) R -= (R >> 10)*11; // Annäherung 1-11/1024 für 2^(-1/64)
		int16_t T6 = 2160580/(n+357)-1639; // Berechnung für 10 kOhm-NTC (bei 25 °C) mit beta=3900 K
		return (T6 > 0 ? T6+3 : T6-2)/6; // Rundung

	}
#endif
	void init_watchdog(void)
	{

		if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
		{

			printf_("watchdog reset!\n");
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			// do not continue here if reset from watchdog
			//while(1){}
			//__HAL_RCC_CLEAR_RESET_FLAGS();
		}
		else
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			printf_("regular reset\n\n");

		}
		// start independent watchdog
		MX_IWDG_Init();


	}

	/* IWDG init function */
	void MX_IWDG_Init(void)
	{
		// RM0008 - Table 96 IWDG timout period in seconds:
		// (IWDG_PRESCALER) * (Period + 1) / f_LSI
		// datasheet STM32F103x4 -> f_LSI = 40'000 Hz ?!
		//
		// 32 * 1000 / 32000 = 1s
		hiwdg.Instance = IWDG;
		hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
		hiwdg.Init.Reload = 1000;
		// start the watchdog timer
		if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

	}
	/* USER CODE END 4 */

	/**
	 * @brief  This function is executed in case of error occurrence.
	 * @param  file: The file name as string.
	 * @param  line: The line in file as a number.
	 * @retval None
	 */
	void _Error_Handler(char *file, int line)
	{
		/* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */
		while(1)
		{
			CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); // Disable PWM in case of an error.
		}
		/* USER CODE END Error_Handler_Debug */
	}

#ifdef  USE_FULL_ASSERT
	/**
	 * @brief  Reports the name of the source file and the source line number
	 *         where the assert_param error has occurred.
	 * @param  file: pointer to the source file name
	 * @param  line: assert_param error line source number
	 * @retval None
	 */
	void assert_failed(uint8_t* file, uint32_t line)
	{
		/* USER CODE BEGIN 6 */
		/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
		/* USER CODE END 6 */
	}
#endif /* USE_FULL_ASSERT */

	/**
	 * @}
	 */

	/**
	 * @}
	 */

	/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
