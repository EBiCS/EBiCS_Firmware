/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <arm_math.h>


/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Hall_1_Pin GPIO_PIN_0
#define Hall_1_GPIO_Port GPIOA
#define Hall_1_EXTI_IRQn EXTI0_IRQn
#define Hall_2_Pin GPIO_PIN_1
#define Hall_2_GPIO_Port GPIOA
#define Hall_2_EXTI_IRQn EXTI1_IRQn
#define Hall_3_Pin GPIO_PIN_2
#define Hall_3_GPIO_Port GPIOA
#define Hall_3_EXTI_IRQn EXTI2_IRQn
#define Throttle_Pin GPIO_PIN_3
#define Throttle_GPIO_Port GPIOA
#define Phase_Current1_Pin GPIO_PIN_4
#define Phase_Current1_GPIO_Port GPIOA
#define Phase_Current_2_Pin GPIO_PIN_5
#define Phase_Current_2_GPIO_Port GPIOA
#define Phase_Current_3_Pin GPIO_PIN_6
#define Phase_Current_3_GPIO_Port GPIOA
#define Temperature_Pin GPIO_PIN_1
#define Temperature_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define LIGHT_Pin GPIO_PIN_9
#define LIGHT_GPIO_Port GPIOB
#define PAS_Pin GPIO_PIN_8
#define PAS_GPIO_Port GPIOB
#define Brake_Pin GPIO_PIN_11
#define Brake_GPIO_Port GPIOA
#define Speed_EXTI5_Pin GPIO_PIN_5
#define Speed_EXTI5_GPIO_Port GPIOB
#define Speed_EXTI5_EXTI_IRQn EXTI9_5_IRQn
#define PAS_EXTI8_Pin GPIO_PIN_8
#define PAS_EXTI8_GPIO_Port GPIOB
#define PAS_EXTI8_EXTI_IRQn EXTI9_5_IRQn



/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */




int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void autodetect();
void runPIcontrol();

extern uint16_t switchtime[3];
extern uint32_t ui32_tim1_counter;
extern uint32_t uint32_PAS_counter;
extern uint8_t throttle_is_set(void);

typedef struct
{
	int16_t       	gain_p;
	int16_t       	gain_i;
	int16_t       	limit_i;
	int16_t       	limit_output;
	int16_t       	recent_value;
	int32_t       	setpoint;
	int32_t       	integral_part;
	int16_t       	max_step;
	int32_t       	out;
	int8_t       	shift;

}PI_control_t;

typedef struct
{

	q31_t       	Voltage;
	uint32_t       	Speed;
	q31_t          	i_d;
	q31_t          	i_q;
	q31_t          	u_d;
	q31_t          	u_q;
	q31_t          	u_abs;
	q31_t          	Battery_Current;
	uint8_t 		hall_angle_detect_flag;
	uint8_t 		char_dyn_adc_state;
	uint8_t 		assist_level;
	uint8_t 		regen_level;
	int8_t         	Temperature;
	int8_t         	system_state;
	int8_t         	gear_state;
	int8_t         	error_state;

}MotorState_t;

typedef struct
{

	uint16_t       	wheel_cirumference;
	uint16_t       	p_Iq;
	uint16_t       	i_Iq;
	uint16_t       	p_Id;
	uint16_t       	i_Id;
	uint16_t       	TS_coeff;
	uint16_t       	PAS_timeout;
	uint16_t       	ramp_end;
	uint16_t       	throttle_offset;
	uint16_t       	throttle_max;
	uint16_t       	gear_ratio;
	uint8_t       	speedLimit;
	uint8_t       	pulses_per_revolution;
	uint16_t       	phase_current_max;
	int16_t       	spec_angle;


}MotorParams_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
