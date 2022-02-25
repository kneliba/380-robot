/*
 * ESC.c
 *
 *  Created on: Feb. 25, 2022
 *      Author: jcwon
 */

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

// drive forward
void drive_forward (TIM_HandleTypeDef *htim, uint32_t speed)
{
	uint32_t pulse_width = 1.5 + (speed/100)*0.5;
	uint32_t ARR = htim->Init.Period;
	uint32_t command = (pulse_width/20)*ARR;

	TIM2->CCR1 = command;
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	TIM2->CCR2 = command;
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}

void stop (TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
}

// turn right

// turn left
