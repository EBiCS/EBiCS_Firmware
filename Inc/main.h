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
#include "settings.h"   // runtime settings block (RS)


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
#define BRAKE_LIGHT_Pin GPIO_PIN_10
#define BRAKE_LIGHT_GPIO_Port GPIOB
#define PAS_Pin GPIO_PIN_8
#define PAS_GPIO_Port GPIOB
#define Brake_Pin GPIO_PIN_11  // put a 15 here for new generation controllers!
#define Brake_GPIO_Port GPIOA
#define Speed_EXTI5_Pin GPIO_PIN_5
#define Speed_EXTI5_GPIO_Port GPIOB
#define Speed_EXTI5_EXTI_IRQn EXTI9_5_IRQn
#define PAS_EXTI8_Pin GPIO_PIN_8
#define PAS_EXTI8_GPIO_Port GPIOB
#define PAS_EXTI8_EXTI_IRQn EXTI9_5_IRQn

//#define NCTE
//#define LEGALFLAG
#ifndef BATTERYVOLTAGE_MAX
#define BATTERYVOLTAGE_MAX 53000
#endif
#ifndef R_TEMP_PULLUP
#define R_TEMP_PULLUP 0
#endif
#ifndef BATTERYVOLTAGE_MIN
#define BATTERYVOLTAGE_MIN (VOLTAGE_MIN*CAL_V)   // cutoff in mV
#endif
#ifndef VOLTAGE_RAMP_MV
#define VOLTAGE_RAMP_MV 1000                   // soft current ramp width at cutoff and full charge voltage
#endif
#ifndef SPEED_TEMP_ADC
#define SPEED_TEMP_ADC 6                       // adcData index of the speed/temperature input: 6 = AD1 (PB1), 5 = AD2 (PB0)
#endif
#ifndef SPEED_TEMP_THRESHOLD
#define SPEED_TEMP_THRESHOLD 100               // ADC value below = speed pulse, above = NTC temperature
#endif
#define INT_TEMP_25 0
#define TORQUE_OFFSET 850
//#define USE_FIX_POSITIONS 1
//Put values from the startup message after autodetect here, if you want to use fix positions. 32bit values for the hall angles!
/*
 * i16_60deg_Hall_flag:
 * bit 0: hallstate 0 detected
 * bit 1: hallstate 7 detected
 * bit 2: 60° hall configuration detected
 * bit 3: hallstate 2 detected
 * bit 4: hallstate 5 detected
 * bit 5: 120° hall configuration detected
 */
#define KV 224
#define HALL_ORDER -1
#define HALL_45 1837291766
#define HALL_51 2565050131
#define HALL_13 3245086636
#define HALL_32 4032497326
#define HALL_26 417566535
#define HALL_64 1109533505

#define HALL_60_46 -1729917165
#define HALL_60_67 -978297870
#define HALL_60_73 -238609040
#define HALL_60_31 417566535
#define HALL_60_10 1169185830
#define HALL_60_4 1861152800

#define CONTROLLER_TEMPERATURE_THRESHOLD 70
#define CONTROLLER_TEMPERATURE_MAX       80
#ifndef MOTOR_TEMPERATURE_THRESHOLD
#define MOTOR_TEMPERATURE_THRESHOLD 100
#endif
#ifndef MOTOR_TEMPERATURE_MAX
#define MOTOR_TEMPERATURE_MAX       130
#endif

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */




// display types added after the Java configurator: fallback if config.h does not define them
#ifndef DISPLAY_TYPE_TONGSHENG
#define DISPLAY_TYPE_TONGSHENG (1<<8)   // Tongsheng VLCD5 / VLCD6 / XH18
#endif
#ifndef DISPLAY_TYPE_VESC
#define DISPLAY_TYPE_VESC (1<<9)        // VESC UART protocol (telemetry + current command)
#endif
uint32_t get_wheeltime_ms(void);
uint8_t motor_is_turning(void);
int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void autodetect();
void runPIcontrol();
void kingmeter_update(void);
void No2_update(void);

extern uint16_t switchtime[3];
extern uint32_t ui32_tim1_counter;
extern uint32_t uint32_PAS_counter;
extern uint8_t throttle_is_set(void);
extern uint8_t brake_is_set(void);
extern void UART_IdleItCallback(void);
extern void get_internal_temp_offset(void);

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
	q31_t 			i_q_setpoint;
	q31_t 			i_d_setpoint;
	q31_t 			i_setpoint_abs;
	int32_t 		i_q_setpoint_temp;
	int32_t 		i_d_setpoint_temp;
	q31_t          	u_d;
	q31_t          	u_q;
	q31_t          	u_abs;
	q31_t          	Battery_Current;
	uint8_t 		hall_angle_detect_flag;
	uint8_t 		char_dyn_adc_state;
	uint8_t 		assist_level;
	uint8_t 		regen_level;
	int16_t         Temperature;
	int16_t         int_Temperature;
	int8_t         	system_state;
	int8_t         	gear_state;
	int8_t         	error_state;
	int8_t 			angle_est;
	int16_t 		KV_detect_flag;

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
	int32_t       	battery_current_max;


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
