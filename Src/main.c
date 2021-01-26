
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


#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


uint32_t ui32_tim1_counter=0;
uint32_t ui32_tim3_counter=0;
uint8_t ui8_hall_state=0;
uint8_t ui8_hall_state_old=0;
uint8_t ui8_hall_case =0;
uint16_t ui16_tim2_recent=0;
uint16_t ui16_timertics=5000; 					//timertics between two hall events for 60° interpolation
uint16_t ui16_reg_adc_value;
uint32_t ui32_reg_adc_value_filter;
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
uint8_t ui8_overflow_flag=0;
uint8_t ui8_slowloop_counter=0;
uint8_t ui8_adc_inj_flag=0;
uint8_t ui8_adc_regular_flag=0;
uint8_t ui8_speedcase=0;
uint8_t ui8_speedfactor=0;
int8_t i8_direction= REVERSE; //for permanent reverse direction
int8_t i8_reverse_flag = 1; //for temporaribly reverse direction

uint8_t ui8_adc_offset_done_flag=0;
uint8_t ui8_print_flag=0;
uint8_t ui8_UART_flag=0;
uint8_t ui8_Push_Assist_flag=0;
uint8_t ui8_UART_TxCplt_flag=1;
uint8_t ui8_PAS_flag=0;
uint8_t ui8_SPEED_flag=0;
uint8_t ui8_BC_limit_flag=0;  //flag for Battery current limitation
uint32_t uint32_PAS_counter= PAS_TIMEOUT+1;
uint32_t uint32_PAS_HIGH_counter= 0;
uint32_t uint32_PAS_HIGH_accumulated= 32000;
uint32_t uint32_PAS_fraction= 100;
uint32_t uint32_SPEED_counter=32000;
uint32_t uint32_PAS=32000;

uint8_t ui8_UART_Counter=0;
int8_t i8_recent_rotor_direction=1;
int16_t i16_hall_order=1;

uint32_t uint32_torque_cumulated=0;
uint32_t uint32_PAS_cumulated=32000;
uint16_t uint16_mapped_throttle=0;
uint16_t uint16_mapped_PAS=0;
uint16_t uint16_half_rotation_counter=0;
uint16_t uint16_full_rotation_counter=0;
int32_t int32_current_target=0;

q31_t q31_t_Battery_Current_accumulated=0;

q31_t q31_rotorposition_absolute;
q31_t q31_rotorposition_hall;
q31_t q31_rotorposition_motor_specific = SPEC_ANGLE;
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
volatile uint16_t adcData[8]; //Buffer for ADC1 Input
//static int8_t angle[256][4];
//static int8_t angle_old;
//q31_t q31_startpoint_conversion = 2048;

//Rotor angle scaled from degree to q31 for arm_math. -180°-->-2^31, 0°-->0, +180°-->+2^31
const q31_t DEG_0 = 0;
const q31_t DEG_plus60 = 715827883;
const q31_t DEG_plus120= 1431655765;
const q31_t DEG_plus180= 2147483647;
const q31_t DEG_minus60= -715827883;
const q31_t DEG_minus120= -1431655765;

const q31_t tics_lower_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*SPEEDLIMIT*10); //tics=wheelcirc*timerfrequency/(no. of hallevents per rev*gear-ratio*speedlimit)*3600/1000000
const q31_t tics_higher_limit = WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*(SPEEDLIMIT+2)*10);
uint32_t uint32_tics_filtered=128000;
uint32_t ui32_Temperature_filter=100;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x01, 0x02, 0x03};


//variables for display communication
#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
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



MotorState_t MS;
MotorParams_t MP;


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


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
void kingmeter_update(void);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
void bafang_update(void);
#endif

static void dyn_adc_state(q31_t angle);
static void set_inj_channel(char state);
void get_standstill_position();
int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t speed_to_tics (uint8_t speed);
int8_t tics_to_speed (uint32_t tics);



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
  MS.assist_level=1;
  MS.regen_level=7;
  MP.pulses_per_revolution = PULSES_PER_REVOLUTION;
  MP.wheel_cirumference = WHEEL_CIRCUMFERENCE;

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
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcData, 7);
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
       if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
         {
           /* Counter Enable Error */
           Error_Handler();
         }

       // Start Timer 3

       if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
            {
              /* Counter Enable Error */
              Error_Handler();
            }


/*
      // HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_CC4);
       __HAL_LOCK(&htim1);
       SET_BIT(TIM1->EGR, TIM_EGR_CC4G);//capture compare ch 4 event
       SET_BIT(TIM1->EGR, TIM_EGR_TG);//Trigger generation
       SET_BIT(TIM1->BDTR, TIM_AUTOMATICOUTPUT_ENABLE);//Trigger generation

       __HAL_UNLOCK(&htim1);
       SET_BIT(ADC1->CR2, ADC_CR2_JEXTTRIG);//external trigger enable

*/

       //Init KingMeter Display
       //Init KingMeter Display
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
       ebics_init();
#endif


    TIM1->CCR1 = 1023; //set initial PWM values
    TIM1->CCR2 = 1023;
    TIM1->CCR3 = 1023;



    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);//Disable PWM

    HAL_Delay(200); //wait for stable conditions

    for(i=0;i<32;i++){
    	while(!ui8_adc_regular_flag){}
    	ui16_ph1_offset+=adcData[2];
    	ui16_ph2_offset+=adcData[3];
    	ui16_ph3_offset+=adcData[4];
    	ui8_adc_regular_flag=0;

    }
    ui16_ph1_offset=ui16_ph1_offset>>5;
    ui16_ph2_offset=ui16_ph2_offset>>5;
    ui16_ph3_offset=ui16_ph3_offset>>5;



   	ui8_adc_offset_done_flag=1;

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
   	printf_("phase current offsets:  %d, %d, %d \n ", ui16_ph1_offset, ui16_ph2_offset, ui16_ph3_offset);
   	autodetect();

#else
   	EE_ReadVariable(EEPROM_POS_SPEC_ANGLE, &MP.spec_angle);

   	// set motor specific angle to value from emulated EEPROM only if valid
   	if(MP.spec_angle!=0xFFFF) {
   		q31_rotorposition_motor_specific = MP.spec_angle<<16;
   		EE_ReadVariable(EEPROM_POS_HALL_ORDER, &i16_hall_order);
   	}
#endif


 // set absolute position to corresponding hall pattern.

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
    printf_("Lishui FOC v0.9 \n ");
    printf_("Motor specific angle:  %d, direction %d \n ", q31_rotorposition_motor_specific, i16_hall_order);
#endif


    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);//Disable PWM

	get_standstill_position();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //PI-control processing
	  if(PI_flag){




		  q31_t_Battery_Current_accumulated -= q31_t_Battery_Current_accumulated>>8;
		  q31_t_Battery_Current_accumulated += ((MS.i_q*MS.u_abs)>>11)*(uint16_t)(CAL_I>>8);

		  MS.Battery_Current = (q31_t_Battery_Current_accumulated>>8)*i8_direction*i8_reverse_flag; //Battery current in mA

		  //Check battery current limit
		  if(MS.Battery_Current>BATTERYCURRENT_MAX) ui8_BC_limit_flag=1;
		  if(MS.Battery_Current<-REGEN_CURRENT_MAX) ui8_BC_limit_flag=1;
		  //reset battery current flag with small hysteresis
		  if(HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)){
			  if(((int32_current_target*MS.u_abs)>>11)*(uint16_t)(CAL_I>>8)<(BATTERYCURRENT_MAX*7)>>3)ui8_BC_limit_flag=0;
		  }
		  else{
			  if(((int32_current_target*MS.u_abs)>>11)*(uint16_t)(CAL_I>>8)>(-REGEN_CURRENT_MAX*7)>>3)ui8_BC_limit_flag=0;
		  }

		  //control iq

		  //if
		  if (!ui8_BC_limit_flag){
			  q31_u_q_temp =  PI_control_i_q(MS.i_q, (q31_t) i8_direction*i8_reverse_flag*int32_current_target);
		  }
		  else{
			  if(HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)){
			  q31_u_q_temp =  PI_control_i_q((MS.Battery_Current>>6)*i8_direction*i8_reverse_flag, (q31_t) (BATTERYCURRENT_MAX>>6)*i8_direction*i8_reverse_flag);
			  }
			  else{
			  q31_u_q_temp =  PI_control_i_q((MS.Battery_Current>>6)*i8_direction*i8_reverse_flag, (q31_t) (-REGEN_CURRENT_MAX>>6)*i8_direction*i8_reverse_flag);
			  }
		  }

		  //Control id
		  q31_u_d_temp = -PI_control_i_d(MS.i_d, 0); //control direct current to zero


		  	//limit voltage in rotating frame, refer chapter 4.10.1 of UM1052
		  MS.u_abs = (q31_t)hypot((double)q31_u_d_temp, (double)q31_u_q_temp); //absolute value of U in static frame



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
	  //display message processing
	  if(ui8_UART_flag){
#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
	  kingmeter_update();
#endif


#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
	  bafang_update();
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
	  ui8_UART_Counter++;
	  if(ui8_UART_Counter>5){
	  check_message(&MS);
	  ui8_UART_Counter=0;
	  }
#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_EBiCS)
	  process_ant_page(&MS, &MP);
#endif

	  ui8_UART_flag=0;
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
		  if(ui16_reg_adc_value>THROTTLE_OFFSET)uint32_torque_cumulated += (ui16_reg_adc_value-THROTTLE_OFFSET);
		  }
	  }

	  //SPEED signal processing
	  if(ui8_SPEED_flag){

		  if(uint32_SPEED_counter>200){
		  MS.Speed = uint32_SPEED_counter;
		  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  uint32_SPEED_counter =0;
		  ui8_SPEED_flag=0;

		  }
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


				if(!HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)){
					if(tics_to_speed(uint32_tics_filtered>>3)>6)int32_current_target=-REGEN_CURRENT; //only apply regen, if motor is turning fast enough
					else int32_current_target=0;
				}
				//next priority: undervoltage protection
				else if(MS.Voltage<VOLTAGE_MIN)int32_current_target=0;
				//next priority: push assist
				else if(ui8_Push_Assist_flag)int32_current_target=PUSHASSIST_CURRENT;
				// last priority normal ride conditiones
				else {

		#ifdef TS_MODE //torque-sensor mode
					//calculate current target form torque, cadence and assist level
					int32_current_target = (TS_COEF*(int16_t)(MS.assist_level)* (uint32_torque_cumulated>>5)/uint32_PAS)>>8; //>>5 aus Mittelung �ber eine Kurbelumdrehung, >>8 aus KM5S-Protokoll Assistlevel 0..255

					//limit currest target to max value
					if(int32_current_target>PH_CURRENT_MAX) int32_current_target = PH_CURRENT_MAX;
					//set target to zero, if pedals are not turning
					if(uint32_PAS_counter > PAS_TIMEOUT){
						int32_current_target = 0;
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

		#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U)
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

				int32_current_target = (int32_current_target * ui8_speedfactor)>>8;

#endif

#ifdef THROTTLE_OVERRIDE

				  // read in throttle for throttle override
				  uint16_mapped_throttle = map(ui16_reg_adc_value, THROTTLE_OFFSET , THROTTLE_MAX, 0, PH_CURRENT_MAX);
				  //check for throttle override
				  if(uint16_mapped_PAS>uint16_mapped_throttle)   {

				    if (uint32_PAS_counter < PAS_TIMEOUT) int32_current_target= uint16_mapped_PAS;		//set current target in torque-simulation-mode, if pedals are turning
					  else  {
						  int32_current_target= 0;//pedals are not turning, stop motor
						  uint32_PAS_cumulated=32000;
						  uint32_PAS=32000;
					  }
				  }
				  else int32_current_target = uint16_mapped_throttle;//throttle override: set recent throttle value as current target
#endif

				//ramp down current at speed limit
			  int32_current_target=map(uint32_tics_filtered>>3,tics_higher_limit,tics_lower_limit,0,int32_current_target);

				//If the motor temperature is between 100 and 120� current target is reduced linearly to up to 25% at 120�C
				  if(MS.Temperature>100) int32_current_target=map(MS.Temperature,100,120,int32_current_target,int32_current_target>2);
				  if(MS.Temperature>120) int32_current_target=0; //Motor stop if temperature is higher than 120�


			} //end else for normal riding

//------------------------------------------------------------------------------------------------------------
				//enable PWM if power is wanted
	  if (int32_current_target>0&&!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){
		  SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		  uint16_half_rotation_counter=0;
		  uint16_full_rotation_counter=0;
		    TIM1->CCR1 = 1023; //set initial PWM values
		    TIM1->CCR2 = 1023;
		    TIM1->CCR3 = 1023;
		  __HAL_TIM_SET_COUNTER(&htim2,0); //reset tim2 counter
		  ui16_timertics=20000; //set interval between two hallevents to a large value
		  i8_recent_rotor_direction=i8_direction*i8_reverse_flag;
		  get_standstill_position();
	  }


//----------------------------------------------------------------------------------------------------------------------------------------------------------
	  //slow loop procedere @16Hz, for LEV standard every 4th loop run, send page,
	  if(ui32_tim3_counter>500){

		  //MS.Temperature = adcData[6]*41>>8; //0.16 is calibration constant: Analog_in[10mV/°C]/ADC value. Depending on the sensor LM35)
		  ui32_Temperature_filter -= ui32_Temperature_filter>>5;
		  ui32_Temperature_filter += adcData[6]; //get Temperature value from AD1
		  MS.Temperature = (ui32_Temperature_filter>>5)*41>>8;//0.16 is calibration constant: Analog_in[10mV/�C]/ADC value. Depending on the sensor LM35)
		  MS.Voltage=adcData[0];
		  if(uint32_SPEED_counter>127999)MS.Speed =128000;

#ifdef INDIVIDUAL_MODES
		  // GET recent speedcase for assist profile
		  if (uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][1]))ui8_speedcase=0;
		  else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][1]) && uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][2]))ui8_speedcase=1;
		  else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][2]) && uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][3]))ui8_speedcase=2;
		  else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][3]) && uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][4]))ui8_speedcase=3;
		  else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][4]))ui8_speedcase=4;

		  ui8_speedfactor = map(uint32_tics_filtered>>3,speed_to_tics(assist_profile[0][ui8_speedcase+1]),speed_to_tics(assist_profile[0][ui8_speedcase]),assist_profile[1][ui8_speedcase+1],assist_profile[1][ui8_speedcase]);


#endif


		  if((uint16_full_rotation_counter>7999||uint16_half_rotation_counter>7999)&&READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){
			  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning

			  get_standstill_position();

		  }

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG && !defined(FAST_LOOP_LOG))
		  //print values for debugging

		   //sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d, %d\r\n", MS.Battery_Current,int32_current_target, MS.Temperature,tics_to_speed(uint32_tics_filtered>>3), (uint16_t)MS.u_abs, uint16_mapped_throttle, MS.i_q, ui32_Temperature_filter);
			sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d, %d\r\n", MS.Battery_Current,int32_current_target, MS.Temperature,tics_to_speed(uint32_tics_filtered>>3), (uint16_t)MS.u_abs, uint16_mapped_throttle, (uint16_mapped_PAS), MS.i_q);

		 // sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d\r\n", MS.i_q,int32_current_target, (int16_t)MS.Battery_Current, MS.i_d, (uint16_t)MS.u_abs,tics_to_speed(uint32_tics_filtered>>3) , (uint16_t)(adcData[6]));
		 // sprintf_(buffer, "%d, %d, %d, %d, %d, %d\r\n",ui8_hall_state,(uint16_t)adcData[1],(uint16_t)adcData[2],(uint16_t)adcData[3],(uint16_t)(adcData[4]),(uint16_t)(adcData[5])) ;

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

			  send_ant_page(ui8_LEV_Page_to_send, &MS, &MP);

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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; //Scan muß für getriggerte Wandlung gesetzt sein
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;// Trigger regular ADC with timer 3 ADC_EXTERNALTRIGCONV_T1_CC1;// // ADC_SOFTWARE_START; //
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
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
  sConfigInjected.AutoInjectedConv = DISABLE; //muß aus sein
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = ui16_ph1_offset;//1900;
  HAL_ADC_Stop(&hadc1); //ADC muß gestoppt sein, damit Triggerquelle gesetzt werden kann.
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
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 128;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64000;
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

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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

#if ((DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER) ||DISPLAY_TYPE==DISPLAY_TYPE_KUNTENG||DISPLAY_TYPE==DISPLAY_TYPE_EBiCS)
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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
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
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3) {
		if(ui32_tim3_counter<32000)ui32_tim3_counter++;
		if (uint32_PAS_counter < PAS_TIMEOUT+1){
			  uint32_PAS_counter++;
			  if(HAL_GPIO_ReadPin(PAS_GPIO_Port, PAS_Pin))uint32_PAS_HIGH_counter++;
		}
		if (uint32_SPEED_counter<128000)uint32_SPEED_counter++;					//counter for external Speedsensor
		if(uint16_full_rotation_counter<8000)uint16_full_rotation_counter++;	//full rotation counter for motor standstill detection
		if(uint16_half_rotation_counter<8000)uint16_half_rotation_counter++;	//half rotation counter for motor standstill detection

	}
}



// regular ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ui32_reg_adc_value_filter -= ui32_reg_adc_value_filter>>4;
#ifdef TQONAD1
	ui32_reg_adc_value_filter += adcData[6]; //get value from AD1
#else
	ui32_reg_adc_value_filter += adcData[1]; //get value from SP
#endif
	ui16_reg_adc_value = ui32_reg_adc_value_filter>>4;

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




	//extrapolate recent rotor position
	ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last event
    if(MS.hall_angle_detect_flag){
	   if (ui16_tim2_recent < ui16_timertics && !ui8_overflow_flag){ //prevent angle running away at standstill
		// float with division necessary!

			   //q31_rotorposition_absolute = q31_rotorposition_hall + (q31_t)(i16_hall_order * i8_recent_rotor_direction * (715827883.0*((float)ui16_tim2_recent/(float)ui16_timertics))); //interpolate angle between two hallevents by scaling timer2 tics
			    q31_rotorposition_absolute = q31_rotorposition_hall + (q31_t)(i16_hall_order * i8_recent_rotor_direction * ((10923 * ui16_tim2_recent)/ui16_timertics)<<16); //interpolate angle between two hallevents by scaling timer2 tics, 10923<<16 is 715827883 = 60�

	   }
	   else
	   {ui8_overflow_flag=1;}

    }//end if hall angle detect

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
	FOC_calculation(i16_ph1_current, i16_ph2_current, q31_rotorposition_absolute, (((int16_t)i8_direction*i8_reverse_flag)*int32_current_target), &MS);
	}
	//temp5=__HAL_TIM_GET_COUNTER(&htim1);
	//set PWM

	TIM1->CCR1 =  (uint16_t) switchtime[0];
	TIM1->CCR2 =  (uint16_t) switchtime[1];
	TIM1->CCR3 =  (uint16_t) switchtime[2];



	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	} // end else

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Hall sensor event processing
	if(GPIO_Pin == GPIO_PIN_0||GPIO_Pin == GPIO_PIN_1||GPIO_Pin == GPIO_PIN_2) //check for right interrupt source
	{
	ui8_hall_state = GPIOA->IDR & 0b111; //Mask input register with Hall 1 - 3 bits


	ui8_hall_case=ui8_hall_state_old*10+ui8_hall_state;
	if(MS.hall_angle_detect_flag){ //only process, if autodetect procedere is fininshed
	ui8_hall_state_old=ui8_hall_state;
	}

	ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last hall event


	if(ui16_tim2_recent>100){//debounce
		ui16_timertics = ui16_tim2_recent; //save timertics since last hall event
		uint32_tics_filtered-=uint32_tics_filtered>>3;
		uint32_tics_filtered+=ui16_timertics;
	   __HAL_TIM_SET_COUNTER(&htim2,0); //reset tim2 counter
	   ui8_overflow_flag=0;

	}



	switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
			{
		//6 cases for forward direction
		case 64:
			q31_rotorposition_hall = DEG_plus120*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=1;
			uint16_full_rotation_counter=0;
			break;
		case 45:
			q31_rotorposition_hall = DEG_plus180*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=1;
			break;
		case 51:
			q31_rotorposition_hall = DEG_minus120*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=1;
			break;
		case 13:
			q31_rotorposition_hall = DEG_minus60*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=1;
			uint16_half_rotation_counter=0;
			break;
		case 32:
			q31_rotorposition_hall = DEG_0*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=1;
			break;
		case 26:
			q31_rotorposition_hall = DEG_plus60*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=1;
			break;

		//6 cases for reverse direction
		case 46:
			q31_rotorposition_hall = DEG_plus120*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=-1;
			break;
		case 62:
			q31_rotorposition_hall = DEG_plus60*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=-1;
			break;
		case 23:
			q31_rotorposition_hall = DEG_0*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=-1;
			uint16_half_rotation_counter=0;
			break;
		case 31:
			q31_rotorposition_hall = DEG_minus60*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=-1;
			break;
		case 15:
			q31_rotorposition_hall = DEG_minus120*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=-1;
			break;
		case 54:
			q31_rotorposition_hall = DEG_plus180*i16_hall_order + q31_rotorposition_motor_specific;
			i8_recent_rotor_direction=-1;
			uint16_full_rotation_counter=0;
			break;

		} // end case


	} //end if

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

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	ui8_UART_flag=1;

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	ui8_UART_TxCplt_flag=1;
}


#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
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

    if(__HAL_TIM_GET_COUNTER(&htim2) < 12000)
    {
        // Adapt wheeltime to match displayed speedo value according config.h setting
    	KM.Tx.Wheeltime_ms = ((MS.Speed>>3)*PULSES_PER_REVOLUTION); //>>3 because of 8 kHz counter frequency, so 8 tics per ms
    }
    else
    {
        KM.Tx.Wheeltime_ms = 64000;
    }


    //KM.Tx.Wheeltime_ms = 25;

    KM.Tx.Error = KM_ERROR_NONE;

    KM.Tx.Current_x10 = (uint16_t) (MS.Battery_Current/100); //MS.Battery_Current is in mA


    /* Receive Rx parameters/settings and send Tx parameters */
    KingMeter_Service(&KM);


    /* Apply Rx parameters */

    MS.assist_level = KM.Rx.AssistLevel;

    if(KM.Rx.Headlight == KM_HEADLIGHT_OFF)
        {
        	HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        else // KM_HEADLIGHT_ON, KM_HEADLIGHT_LOW, KM_HEADLIGHT_HIGH
        {
        	HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        }


    if(KM.Rx.PushAssist == KM_PUSHASSIST_ON)
    {
    	ui8_Push_Assist_flag=1;
    }
    else
    {
    	ui8_Push_Assist_flag=0;
    }



}

#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG)
void bafang_update(void)
{
    /* Prepare Tx parameters */

	if(adcData[0]*CAL_BAT_V>BATTERY_LEVEL_5)battery_percent_fromcapacity=75;
	else if(adcData[0]*CAL_BAT_V>BATTERY_LEVEL_4)battery_percent_fromcapacity=50;
	else if(adcData[0]*CAL_BAT_V>BATTERY_LEVEL_3)battery_percent_fromcapacity=30;
	else if(adcData[0]*CAL_BAT_V>BATTERY_LEVEL_2)battery_percent_fromcapacity=10;
	else if(adcData[0]*CAL_BAT_V>BATTERY_LEVEL_1)battery_percent_fromcapacity=5;
	else battery_percent_fromcapacity=0;


    	BF.Tx.Battery = battery_percent_fromcapacity;


    if(__HAL_TIM_GET_COUNTER(&htim2) < 12000)
    {
        // Adapt wheeltime to match displayed speedo value according config.h setting
        BF.Tx.Wheeltime_ms = WHEEL_CIRCUMFERENCE*216/(MS.Speed*PULSES_PER_REVOLUTION); // Geschwindigkeit ist Weg pro Zeit Radumfang durch Dauer einer Radumdrehung --> Umfang * 8000*3600/(n*1000000) * Skalierung Bafang Display 200/26,6

    }
    else
    {
        BF.Tx.Wheeltime_ms = 0; //64000;
    }


       BF.Tx.Power = MS.i_q*MS.Voltage;


    /* Receive Rx parameters/settings and send Tx parameters */
    Bafang_Service(&BF,1);



    /* Apply Rx parameters */

//No headlight supported on my controller hardware.
    if(BF.Rx.Headlight)
    {
       // digitalWrite(lights_pin, 0);
    }
    else
    {
       // digitalWrite(lights_pin, 1);
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
		MS.char_dyn_adc_state = 1; // -90° .. +30°: Phase C at high dutycycles
		if(switchtime[2]>1500)TIM1->CCR4 =  switchtime[2]-TRIGGER_OFFSET_ADC;
		else TIM1->CCR4 = TRIGGER_DEFAULT;
	}

	if (switchtime[0]>switchtime[1] && switchtime[0]>switchtime[2]) {
		MS.char_dyn_adc_state = 2; // +30° .. 150° Phase A at high dutycycles
		if(switchtime[0]>1500)TIM1->CCR4 =  switchtime[0]-TRIGGER_OFFSET_ADC;
		else TIM1->CCR4 = TRIGGER_DEFAULT;
	}

	if (switchtime[1]>switchtime[0] && switchtime[1]>switchtime[2]){
		MS.char_dyn_adc_state = 3; // +150 .. -90° Phase B at high dutycycles
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

void autodetect(){
	SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
   	MS.hall_angle_detect_flag=0; //set uq to contstant value in FOC.c for open loop control
   	q31_rotorposition_absolute=1<<31;
   	uint8_t zerocrossing=0;
   	q31_t diffangle=0;
   	HAL_Delay(5);
   	for(i=0;i<1080;i++){
   		q31_rotorposition_absolute+=11930465; //drive motor in open loop with steps of 1�
   		HAL_Delay(5);
   		if (q31_rotorposition_absolute>-60&&q31_rotorposition_absolute<60){
   			switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
   						{
   					//6 cases for forward direction
   					case 64:
   						zerocrossing = 45;
   						diffangle=DEG_plus180;
   						break;
   					case 45:
   						zerocrossing = 51;
   						diffangle=DEG_minus120;
   						break;
   					case 51:
   						zerocrossing = 13;
   						diffangle=DEG_minus60;
   						break;
   					case 13:
   						zerocrossing = 32;
   						diffangle=DEG_0;
   						break;
   					case 32:
   						zerocrossing = 26;
   						diffangle=DEG_plus60;
   						break;
   					case 26:
   						zerocrossing = 64;
   						diffangle=DEG_plus120;
   						break;

   					//6 cases for reverse direction
   					case 46:
   						zerocrossing = 62;
   						diffangle=-DEG_plus60;
   						break;
   					case 62:
   						zerocrossing = 23;
   						diffangle=-DEG_0;
   						break;
   					case 23:
   						zerocrossing = 31;
   						diffangle=-DEG_minus60;
   						break;
   					case 31:
   						zerocrossing = 15;
   						diffangle=-DEG_minus120;
   						break;
   					case 15:
   						zerocrossing = 54;
   						diffangle=-DEG_plus180;
   						break;
   					case 54:
   						zerocrossing = 46;
   						diffangle=-DEG_plus120;
   						break;

   					} // end case


   		}

   		if(ui8_hall_state_old!=ui8_hall_state){
   		printf_("angle: %d, hallstate:  %d, hallcase %d \n",(int16_t)(((q31_rotorposition_absolute>>23)*180)>>8), ui8_hall_state , ui8_hall_case);

   		if(ui8_hall_case==zerocrossing)
   		{
   			q31_rotorposition_motor_specific = q31_rotorposition_absolute-diffangle-(1<<31);
   		}


   		ui8_hall_state_old=ui8_hall_state;
   		}
   	}
    HAL_FLASH_Unlock();
    EE_WriteVariable(EEPROM_POS_SPEC_ANGLE, q31_rotorposition_motor_specific>>16);
    if(i8_recent_rotor_direction == 1){
    	EE_WriteVariable(EEPROM_POS_HALL_ORDER, 1);
    	i16_hall_order = 1;
    }
    else{
    	EE_WriteVariable(EEPROM_POS_HALL_ORDER, -1);
    	i16_hall_order = -1;
    }

    HAL_FLASH_Lock();

   	MS.hall_angle_detect_flag=1;
#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
    printf_("Motor specific angle:  %d, direction %d \n ", (int16_t)(((q31_rotorposition_motor_specific>>23)*180)>>8), i16_hall_order);
#endif

    HAL_Delay(5);
}

void get_standstill_position(){
	  HAL_Delay(100);
	  HAL_GPIO_EXTI_Callback(GPIO_PIN_0); //read in initial rotor position
		switch (ui8_hall_state)
			{
			//6 cases for forward direction
			case 2:
				q31_rotorposition_hall = DEG_0 + q31_rotorposition_motor_specific;
				break;
			case 6:
				q31_rotorposition_hall = DEG_plus60 + q31_rotorposition_motor_specific;
				break;
			case 4:
				q31_rotorposition_hall = DEG_plus120 + q31_rotorposition_motor_specific;
				break;
			case 5:
				q31_rotorposition_hall = DEG_plus180 + q31_rotorposition_motor_specific;
				break;
			case 1:
				q31_rotorposition_hall = DEG_minus120 + q31_rotorposition_motor_specific;
				break;
			case 3:
				q31_rotorposition_hall = DEG_minus60 + q31_rotorposition_motor_specific;
				break;

			}

		 q31_rotorposition_absolute = q31_rotorposition_hall;
}

int32_t speed_to_tics (uint8_t speed){
	return WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*speed*10);
}

int8_t tics_to_speed (uint32_t tics){
	return WHEEL_CIRCUMFERENCE*5*3600/(6*GEAR_RATIO*tics*10);;
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
