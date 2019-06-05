
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
#include "math.h"



/* USER CODE BEGIN Includes */

// Lishui BLCD FOC Open Source Firmware
// Board uses IRS2003 half bridge drivers, this need inverted pulses for low-side Mosfets, deadtime is generated in driver
// This firmware bases on the ST user manual UM1052
// It uses the OpenSTM32 workbench (SW4STM32 toolchain)
// Basic peripheral setup was generated with CubeMX

#include "print.h"
#include "FOC.h"
#include "display_kingmeter.h"
#include "config.h"
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
uint8_t ui8_hall_state=0;
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
uint8_t ui8_overflow_flag=0;
uint8_t ui8_slowloop_flag=0;
uint8_t ui8_print_flag=0;
uint8_t ui8_UART_flag=0;
uint8_t ui8_PAS_flag=0;
uint8_t ui8_SPEED_flag=0;
uint32_t uint32_PAS_counter= PAS_TIMEOUT+1;
uint32_t uint32_SPEED_counter=32000;
uint32_t uint32_PAS=32000;
uint32_t uint32_SPEED=32000;
uint32_t uint32_torque_cumulated=0;
uint16_t uint16_mapped_throttle=0;
uint16_t uint16_mapped_PAS=0;
uint16_t uint16_current_target=0;

q31_t q31_rotorposition_absolute;
q31_t q31_rotorposition_hall;
//int16_t i16_sinus=0;
//int16_t i16_cosinus=0;
char buffer[100];
char char_dyn_adc_state=1;
char char_dyn_adc_state_old=1;
q31_t	q31_u_abs=0;
volatile static q31_t q31_teta_obs;
q31_t q31_delta_teta;

q31_t switchtime[3];
uint16_t adcData[8];
//static int8_t angle[256][4];
static int8_t angle_old;
//q31_t q31_startpoint_conversion = 2048;

//Rotor angle scaled from degree to q31 for arm_math. -180°-->-2^31, 0°-->0, +180°-->+2^31
const q31_t DEG_0 = 0;
const q31_t DEG_plus60 = 715827883;
const q31_t DEG_plus120= 1431655765;
const q31_t DEG_plus180= 2147483647;
const q31_t DEG_minus60= -715827883;
const q31_t DEG_minus120= -1431655765;

//variables for display communication

KINGMETER_t KM;
int16_t battery_percent_fromcapacity = 11; 			//Calculation of used watthours not implemented yet
int16_t wheel_time = 1000;							//duration of one wheel rotation for speed calculation
int16_t current_display;							//pepared battery current for display
int16_t throttle_stat;								//throttle value, not linked to ADC-Value yet
int16_t poti_stat;									//scaled assist level



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

void kingmeter_update(void);
static void dyn_adc_state(q31_t angle);
static void set_inj_channel(char state);
int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);


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
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcData, 5);
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


    TIM1->CCR1 = 1024; //set initial PWM values
    TIM1->CCR2 = 1024;
    TIM1->CCR3 = 1024;

    TIM1->CCR4 = 1014; //ADC sampling just before timer overflow (just before middle of PWM-Cycle)
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
       KingMeter_Init (&KM);





    HAL_GPIO_EXTI_Callback(GPIO_PIN_0); //read in initial rotor position
    q31_rotorposition_absolute = q31_rotorposition_hall; // set absolute position to corresponding hall pattern.


    printf_("Lishui FOC v0.0 \r\n");




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //PI-control processing
	  if(PI_flag){


		  q31_u_q =  PI_control_i_q(q31_i_q_fil>>3, (q31_t) uint16_current_target);



		  	//Control id
		  	q31_u_d = -PI_control_i_d(q31_i_d_fil>>3, 0); //control direct current to zero

		  	//limit voltage in rotating frame, refer chapter 4.10.1 of UM1052

		  	q31_u_abs = hypot(q31_u_q, q31_u_d); //absolute value of U in static frame



		  	if (q31_u_abs > _U_MAX){
		  		q31_u_q = (q31_u_q*_U_MAX)/q31_u_abs; //division!
		  		q31_u_d = (q31_u_d*_U_MAX)/q31_u_abs; //division!
		  		//temp4=1;
		  	}
		  	else //temp4=0;
		  	PI_flag=0;
	  }
	  if(Obs_flag){
		  q31_delta_teta = PI_control_e_d(q31_e_d_obs, 0);
		  Obs_flag=0;
	  }

	  //display message processing
	  if(ui8_UART_flag){
	  kingmeter_update();
	  ui8_UART_flag=0;
	  }

	  //PAS signal processing
	  if(ui8_PAS_flag){
		  uint32_PAS = uint32_PAS_counter;
		  uint32_PAS_counter =0;
		  ui8_PAS_flag=0;
		  //read in and sum up torque-signal within one crank revolution (for sempu sensor 32 PAS pulses/revolution, 2^5=32)
		  uint32_torque_cumulated -= uint32_torque_cumulated>>5;
		  uint32_torque_cumulated += (ui16_reg_adc_value-THROTTE_OFFSET);
	  }

	  //SPEED signal processing
	  if(ui8_SPEED_flag){
		  uint32_SPEED = uint32_SPEED_counter;
		  uint32_SPEED_counter =0;
		  ui8_SPEED_flag=0;
	  }

	  //throttle and PAS current target setting


#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
	  uint16_mapped_PAS = map(uint32_PAS, RAMP_END, PAS_TIMEOUT, (PH_CURRENT_MAX*(int32_t)(KM.Rx.AssistLevel-1))>>2, 0); // level in range 1...5
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U)
	  uint16_mapped_PAS = map(uint32_PAS, RAMP_END, PAS_TIMEOUT, (PH_CURRENT_MAX*(int32_t)(KM.Rx.AssistLevel))>>8, 0); // level in range 0...255
#endif
#ifdef TS_MODE //torque-sensor mode

	  uint16_current_target = (TS_COEF*(int16_t)(KM.Rx.AssistLevel)* (uint32_torque_cumulated>>5)/uint32_PAS)>>8;
	  if(uint16_current_target>PH_CURRENT_MAX) uint16_current_target = PH_CURRENT_MAX;
	  if(uint32_PAS_counter > PAS_TIMEOUT) uint16_current_target = 0;
	  //uint16_current_target = 0;

#else
	  uint16_mapped_throttle = map(ui16_reg_adc_value, THROTTE_OFFSET ,4096, 0, PH_CURRENT_MAX);
	  if (uint16_mapped_PAS>uint16_mapped_throttle)

	  {
		  if (uint32_PAS_counter < PAS_TIMEOUT) uint16_current_target= uint16_mapped_PAS;
		  else uint16_current_target= 0;
	  }
	  else uint16_current_target = uint16_mapped_throttle;

#endif

	  //enable PWM output, if power is wanted
	  if (uint16_current_target>0)TIM1->BDTR |= 1L<<15; //set MOE bit

	  if(q31_rotorposition_absolute>>24!=angle_old){
	  			angle_old = q31_rotorposition_absolute>>24;

	  			//buffer[0]=angle_old;
	  			//buffer[1]=(q31_teta_obs>>24);
	  			buffer[0]=(char)(((atan2((double)temp2,(double)temp1)+3.1416)*40));
	  			buffer[1]=(char)(((atan2((double)temp4,(double)temp3)+3.1416)*40));
	  			buffer[2]=(char)(angle_old+128);
	  			//buffer[3]=(char)temp4;
	  			//buffer[4]=0xFF;

	  		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&buffer, 3);


	  }



	  //print values for debugging
	  	  if(ui32_tim1_counter>800){

/*
	  		sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d, %d\r\n", (int16_t)q31_i_q_fil>>3, (int16_t)((q31_i_q_fil>>3)*q31_u_abs/_T) , uint16_current_target, (int16_t)q31_u_abs,  (int16_t)temp1, (int16_t)temp2,(int16_t)q31_e_d_obs,(int16_t) q31_u_d);
	  	//	sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d, %d\r\n",(uint16_t)adcData[0],(uint16_t)adcData[1],(uint16_t)adcData[2],(uint16_t)adcData[3],(uint16_t)(adcData[4]),(uint16_t)(adcData[5]),(uint16_t)(adcData[6]),(uint16_t)(adcData[7])) ;
	  	 i=0;
		  while (buffer[i] != '\0')
		  {i++;}
		 HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&buffer, i);*/
	  	/* if (ui8_print_flag==1){
	  		ui8_print_flag=2;

	  		 for(j=0; j<255; j++){
	  		sprintf_(buffer, "%d, %d, %d, %d\r\n", angle[j][0] , angle[j][1], angle[j][2], angle[j][3]);
		  i=0;
		  while (buffer[i] != '\0')
		  {i++;}
		 HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&buffer, i);
		 HAL_Delay(5);

	  		 }
	  	 }*/
		  ui32_tim1_counter=0;
		  ui8_print_flag=0;
	  	  }


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
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;// // ADC_SOFTWARE_START; //
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
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
  sConfigInjected.InjectedOffset = OFFSET_A;//1900;
  HAL_ADC_Stop(&hadc1); //ADC muß gestoppt sein, damit Triggerquelle gesetzt werden kann.
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_3;
sConfig.Rank = ADC_REGULAR_RANK_2;
sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
_Error_Handler(__FILE__, __LINE__);
}
/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_4;
sConfig.Rank = ADC_REGULAR_RANK_3;
sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
_Error_Handler(__FILE__, __LINE__);
}
/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_5;
sConfig.Rank = ADC_REGULAR_RANK_4;
sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
_Error_Handler(__FILE__, __LINE__);
}
/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_6;
sConfig.Rank = ADC_REGULAR_RANK_5;
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
  sConfigInjected.InjectedOffset = OFFSET_B;//	1860;
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
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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
  sBreakDeadTimeConfig.DeadTime = 0;
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

/* TIM3 init function 8kHz interrupt frequency for FOC calculation */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4*_T;
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

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  huart1.Init.BaudRate = 56000;//9600;
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

  /*Configure GPIO pin : PAS_Pin */ // für Debug PAS als Ausgang
 /* GPIO_InitStruct.Pin = PAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PAS_GPIO_Port, &GPIO_InitStruct);*/

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





//Timer1 CC Channel4
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		  ui32_tim1_counter++;
		  if (uint32_PAS_counter < PAS_TIMEOUT+1)uint32_PAS_counter++;
		  uint32_SPEED_counter++;
		  temp5=__HAL_TIM_GET_COUNTER(&htim1);


	  }

}

//Timer2 Counter for speed measurement, callback handling not necessary
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);


		// call FOC procedure
		FOC_calculation(i16_ph1_current, i16_ph2_current, q31_rotorposition_absolute, uint16_current_target);

		q31_teta_obs += q31_delta_teta;

		//set PWM
		TIM1->CCR1 =  (uint16_t) switchtime[0];
		TIM1->CCR2 =  (uint16_t) switchtime[1];
		TIM1->CCR3 =  (uint16_t) switchtime[2];

		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	}
}

// regular ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ui32_reg_adc_value_filter -= ui32_reg_adc_value_filter>>4;
	ui32_reg_adc_value_filter += adcData[1]; //HAL_ADC_GetValue(hadc);
	ui16_reg_adc_value = ui32_reg_adc_value_filter>>4;


	//temp5=ui16_reg_adc_value-665;
}

//injected ADC

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//for oszi-check of used time in FOC procedere
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	//read in phase currents
	//i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	//i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);


	switch (char_dyn_adc_state) //read in according to state
		{
		case 1: //Phase C at high dutycycles, read from A+B directly
			{
				i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
			}
			break;
		case 2: //Phase A at high dutycycles, read from B+C (A = -B -C)
			{
				i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
				i16_ph1_current = -i16_ph2_current-HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

			}
			break;
		case 3: //Phase B at high dutycycles, read from A+C (B=-A-C)
			{
				i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				i16_ph2_current = -i16_ph1_current-HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
			}
			break;

		case 0: //timeslot too small for ADC
			{
				//do nothing
			}
			break;


		} // end case






	//extrapolate recent rotor position
	ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last event

	if (ui16_tim2_recent < ui16_timertics && !ui8_overflow_flag){ //prevent angle running away at standstill
		// float with division necessary!
		q31_rotorposition_absolute = q31_rotorposition_hall + (q31_t) (715827883.0*((float)ui16_tim2_recent/(float)ui16_timertics)); //interpolate angle between two hallevents by scaling timer2 tics

//debugging
	/*	if(q31_rotorposition_absolute>>24!=angle_old){
			angle_old = q31_rotorposition_absolute>>24;
			if(ui8_print_flag==0){
				angle[ui32_counter++][0] = q31_rotorposition_absolute>>24;
				angle[ui32_counter][1] = temp5;
				angle[ui32_counter][2] = temp6;
				angle[ui32_counter][3] = temp1;

				if(ui32_counter>255){
						ui32_counter=0;
						ui8_print_flag=1;
					}
			}
		}*/
	}
	else
	{ui8_overflow_flag=1;

	}

	//q31_rotorposition_absolute=-(int32_t)((float)ui16_reg_adc_value/1580.0*2147483647.0);


	// float with division! For debugging, not necessary
	//temp2=(q31_t)((float)q31_rotorposition_absolute/2147483648.0*180.0);


	//get the Phase with highest duty cycle for dynamic phase current reading
	dyn_adc_state(q31_rotorposition_absolute);
	//set the according injected channels to read current at Low-Side active time

	if (char_dyn_adc_state!=char_dyn_adc_state_old){
		set_inj_channel(char_dyn_adc_state);
		char_dyn_adc_state_old = char_dyn_adc_state;
		}

/*
	//uint16_current_target=0;
	// call FOC procedure
	FOC_calculation(i16_ph1_current, i16_ph2_current, q31_rotorposition_absolute, uint16_current_target);

	q31_teta_obs += q31_delta_teta;

	//temp5=__HAL_TIM_GET_COUNTER(&htim1);
	//set PWM
	TIM1->CCR1 =  (uint16_t) switchtime[0];
	TIM1->CCR2 =  (uint16_t) switchtime[1];
	TIM1->CCR3 =  (uint16_t) switchtime[2];
	//TIM1->CCR4 =  (uint16_t) q31_startpoint_conversion;


	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	//read in timer for indication of processor load
	//temp4=__HAL_TIM_GET_COUNTER(&htim1);*/

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Hall sensor event processing
	if(GPIO_Pin == GPIO_PIN_0||GPIO_Pin == GPIO_PIN_1||GPIO_Pin == GPIO_PIN_2) //check for right interrupt source
	{
	ui8_hall_state = GPIOA->IDR & 0b111; //Mask input register with Hall 1 - 3 bits

	ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last hall event


	if(ui16_tim2_recent>100){//debounce
		ui16_timertics = ui16_tim2_recent; //save timertics since last hall event
	   __HAL_TIM_SET_COUNTER(&htim2,0); //reset tim2 counter
	   ui8_overflow_flag=0;

	}
	//temp6=ui16_timertics;
	switch (ui8_hall_state) //according to UM1052 Fig 57, Page 72, 120° setup
	{
	case 5: //0°
		q31_rotorposition_hall = DEG_0 + SPEC_ANGLE;
		break;
	case 1: //60°
		q31_rotorposition_hall = DEG_plus60 + SPEC_ANGLE;
		break;
	case 3: //120°
		q31_rotorposition_hall = DEG_plus120 + SPEC_ANGLE;
		break;
	case 2: //180°
		q31_rotorposition_hall = DEG_plus180 + SPEC_ANGLE; 	//overflow doesn't matter?!
		break;
	case 6: //240°-->-120°
		q31_rotorposition_hall = DEG_minus120 + SPEC_ANGLE;
		break;
	case 4: //300°-->-60°
		q31_rotorposition_hall = DEG_minus60 + SPEC_ANGLE;
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
		ui8_SPEED_flag = 1;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	ui8_UART_flag=1;

}

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
        KM.Tx.Wheeltime_ms = (ui16_timertics>>1);
    }
    else
    {
        KM.Tx.Wheeltime_ms = 64000;
    }


    //KM.Tx.Wheeltime_ms = 25;

    KM.Tx.Error = KM_ERROR_NONE;

    KM.Tx.Current_x10 = (uint16_t) (temp1>>1);


    /* Receive Rx parameters/settings and send Tx parameters */
    KingMeter_Service(&KM);


    /* Apply Rx parameters */

    #ifdef SUPPORT_LIGHTS_SWITCH
    if(KM.Rx.Headlight == KM_HEADLIGHT_OFF)
    {
        digitalWrite(lights_pin, 0);
    }
    else // KM_HEADLIGHT_ON, KM_HEADLIGHT_LOW, KM_HEADLIGHT_HIGH
    {
        digitalWrite(lights_pin, 1);
    }
    #endif

    if(KM.Rx.PushAssist == KM_PUSHASSIST_ON)
    {
        #if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U)
        throttle_stat = map(KM.Rx.AssistLevel, 0, 255, 0,1023);
        #else
        throttle_stat = 200;
        #endif
    }
    else
    {
        throttle_stat = 0;
        poti_stat     = map(KM.Rx.AssistLevel, 0, 255, 0,1023);
    }


    /* Shutdown in case we received no message in the last 3s */
/*
    if((millis() - KM.LastRx) > 3000)
    {
        poti_stat     = 0;
        throttle_stat = 0;
        #if HARDWARE_REV >=2
        save_shutdown();
        #endif
    }*/
}

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
		char_dyn_adc_state = 1; // -90° .. +30°: Phase C at high dutycycles
		if(q31_u_abs>1700){
			/*if ((switchtime[2]-switchtime[0]<ADC_DUR)||(switchtime[2]-switchtime[1])<ADC_DUR){
				//char_dyn_adc_state = 0; //time frame to small for ADC
			}
			else*/ TIM1->CCR4 = switchtime[2]-ADC_DUR; //set startpoint of ADC to 350 tics before highest phase is switched off
		}
		else TIM1->CCR4 =_T-1; //set startpoint of ADC to timer overflow
	}
	if (switchtime[0]>switchtime[1] && switchtime[0]>switchtime[2]) {
		char_dyn_adc_state = 2; // +30° .. 150° Phase A at high dutycycles
		if(q31_u_abs>1700){
			/*if ((switchtime[0]-switchtime[1]<ADC_DUR)||(switchtime[0]-switchtime[2])<ADC_DUR){
				//char_dyn_adc_state = 0; //time frame to small for ADC
			}
			else*/ TIM1->CCR4 = switchtime[0]-ADC_DUR; //set startpoint of ADC to 350 tics before highest phase is switched off
		}
		else TIM1->CCR4 =_T-1; //set startpoint of ADC to timer overflow
	}
	if (switchtime[1]>switchtime[0] && switchtime[1]>switchtime[2]){
		char_dyn_adc_state = 3; // +150 .. -90° Phase B at high dutycycles
		if(q31_u_abs>1700){
			/*if ((switchtime[1]-switchtime[0]<ADC_DUR)||(switchtime[1]-switchtime[2])<ADC_DUR){
				//char_dyn_adc_state = 0; //time frame to small for ADC
			}
			else*/ TIM1->CCR4 = switchtime[1]-ADC_DUR; //set startpoint of ADC to 350 tics before highest phase is switched off
		}
		else TIM1->CCR4 =_T-1; //set startpoint of ADC to timer overflow
	}
}

static void set_inj_channel(char state){
	switch (state)
	{
	case 1: //Phase C at high dutycycles, read current from phase A + B
		 {
			 ADC1->JSQR=0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
			 ADC1->JOFR1 = OFFSET_A;
			 ADC2->JSQR=0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
			 ADC2->JOFR1 = OFFSET_B;
		 }
			break;
	case 2: //Phase A at high dutycycles, read current from phase C + B
			 {
				 ADC1->JSQR=0b00110000000000000000; //ADC1 injected reads phase C, JSQ4 = 0b00110, decimal 6
				 ADC1->JOFR1 = OFFSET_C;
				 ADC2->JSQR=0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
				 ADC2->JOFR1 = OFFSET_B;
			 }
				break;

	case 3: //Phase B at high dutycycles, read current from phase A + C
			 {
				 ADC1->JSQR=0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
				 ADC1->JOFR1 = OFFSET_A;
				 ADC2->JSQR=0b00110000000000000000; //ADC2 injected reads phase C, JSQ4 = 0b00110, decimal 6
				 ADC2->JOFR1 = OFFSET_C;
			 }
				break;


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
