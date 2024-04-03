/*
 * m365
 *
 * Copyright (c) 2021 Francois Deslandes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "button_processing.h"
#include "main.h"
#include "config.h"
#include "stm32f1xx_hal.h"

volatile uint32_t main_loop_counter;
static uint8_t power_button_state = 0;
eButtonEvent button_event = NO_PRESS ;
UART_HandleTypeDef huart3;

void buttonState() {
    //static const uint32_t DEBOUNC = 100 ;
    static uint16_t buttoncounter;
    if(!HAL_GPIO_ReadPin( PWR_BTN_GPIO_Port, PWR_BTN_Pin ))buttoncounter++;
    else{
    	if (buttoncounter>4000){
    		button_event = LONG_PRESS;
    	}
    	else if (buttoncounter>500){
    		button_event = SINGLE_PRESS;
    	}
    	else button_event = NO_PRESS ;
    	buttoncounter=0;
    	}
    }




//eButtonEvent getButtonEvent()
//{
//    static const uint32_t DOUBLE_GAP_MILLIS_MAX 	= 250;
//    static const uint32_t SINGLE_PRESS_MILLIS_MAX 	= 800;
//    static const uint32_t LONG_PRESS_MILLIS_MAX 	= 5000;
//
//    static uint32_t button_down_ts = 0 ;
//    static uint32_t button_up_ts = 0 ;
//    static bool double_pending = false ;
//    static bool button_down = false ; ;
//
//
//    uint32_t now = HAL_GetTick() ;
//
//    if( button_down != buttonState() ) {
//        button_down = !button_down ;
//        if( button_down ) {
//            button_down_ts = now ;
//        } else {
//            button_up_ts = now ;
//            if( double_pending ) {
//                button_event = DOUBLE_PRESS ;
//                double_pending = false ;
//            }
//            else {
//                double_pending = true ;
//            }
//        }
//    }
//
//   // uint32_t diff =  button_up_ts - button_down_ts;
//    if (!button_down && double_pending && now - button_up_ts > DOUBLE_GAP_MILLIS_MAX) {
//    	double_pending = false ;
//    	button_event = SINGLE_PRESS ;
//	} else if (button_down && now -button_down_ts >= SINGLE_PRESS_MILLIS_MAX && now -button_down_ts <= LONG_PRESS_MILLIS_MAX) {
//		double_pending = false ;
//		button_event = LONG_PRESS ;
//	} else if (button_down && now - button_down_ts > LONG_PRESS_MILLIS_MAX) {
//		double_pending = false ;
//		button_event = VERY_LONG_PRESS ;
//	}
//
//    return button_event ;
//}

void checkButton(MotorParams_t *MP,MotorState_t *MS) {
	/* Infinite loop */
	  buttonState();
	  //if(MS->shutdown>85&&(MS->mode>>4)) power_control(DEV_PWR_OFF);

			switch( button_event ){
				  case NO_PRESS : break ;
				  case LONG_PRESS : {
					  MS->light = !MS->light;
					  if (MS->light)HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin,SET);
					  else HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin,RESET);
					  MS->beep = 1;

				  } break ;
//				  case VERY_LONG_PRESS :   {
//					  MS->mode &= ~(1 << 4); //clear "off" (bit 4)
//					  MS->shutdown=0;
//					  autodetect();
//					 // commands_printf("LONG_PRESS");
//				  } break ;
//				  case LONG_PRESS :		{
//					  MS->mode |= (1 << 4); //set "off" (bit 4)
//
//					  if(MS->shutdown==0){
//						  MS->shutdown=1;
//						  MS->beep = 1;
//					  }
//
//				  } break ;

				  case SINGLE_PRESS : {
					 // commands_printf("DOUBLE_PRESS");
					  MS->mode=MS->mode+2;
					  if(MS->mode>4)MS->mode=0;
					  set_mode(MP,MS);
					  MS->beep = 1;

				  } break ;
			 }
}

void PWR_init() {
	/* Check button pressed state at startup */
	//power_button_state = buttonState();

    /* Power ON board temporarily, ultimate decision to keep hardware ON or OFF is made later */
	power_control(DEV_PWR_ON);

	}

void power_control(uint8_t pwr)
{
	if(pwr == DEV_PWR_ON) {
		/* Turn the PowerON line high to keep the board powered on even when
		 * the power button is released
		 */
		HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_SET);
	} else if(pwr == DEV_PWR_OFF) {

		//motors_free(0, NULL);
		//sleep_x_ticks(2000);
		//stop_motors();


		while(HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin));
		HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_RESET);
		while(1);
	} else if(pwr == DEV_PWR_RESTART) {

		//motors_free(0, NULL);
		//sleep_x_ticks(2000);
		//stop_motors();

		/* Restart the system */
		NVIC_SystemReset();
	}
}

void set_mode(MotorParams_t *MP, MotorState_t *MS){

	switch( MS->mode & 0x07){ //look only on the lowest 3 bits
		case eco :{
			MP->phase_current_limit=PH_CURRENT_MAX_ECO/CAL_I;
			MP->speed_limit=SPEEDLIMIT_ECO;

			} break ;
		case normal :{
			MP->phase_current_limit=PH_CURRENT_MAX_NORMAL/CAL_I;
			MP->speed_limit=SPEEDLIMIT_NORMAL;

			} break ;
		case sport :{
			MP->phase_current_limit=PH_CURRENT_MAX_SPORT/CAL_I;
			MP->speed_limit=SPEEDLIMIT_SPORT;

			} break ;

	}
	calculate_tic_limits();
}


