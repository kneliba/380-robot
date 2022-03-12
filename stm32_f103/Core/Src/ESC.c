#include "ESC.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

// drive forward - speed %
void drive_forward (TIM_HandleTypeDef *htim, uint32_t speed)
{
	uint32_t pulse_width = 1 + (speed/100)*0.5;
	uint32_t ARR = htim->Init.Period;
	uint32_t command = (pulse_width/20)*ARR;

	TIM2->CCR1 = command;
//	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
//	TIM2->CCR2 = command;
//	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}

void stop (TIM_HandleTypeDef *htim)
{
	drive_forward (htim, 0);
}

// turn right
//void turn_right (TIM_HandleTypeDef *htim)
//{
//
//}
