#include "ultrasonic.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f103xb.h"
#include "main.h"

HCSR04_Type Front_US =
{
	FRONT_TRIG_GPIO_Port,
	FRONT_TRIG_Pin,
	TIM3,
	TIM_CHANNEL_2,
	0,
	0,
	0,
	0,
	0
};

HCSR04_Type Side_US =
{
	SIDE_TRIG_GPIO_Port,
	SIDE_TRIG_Pin,
	TIM3,
	TIM_CHANNEL_3,
	0,
	0,
	0,
	0,
	0
};

void HCSR04_Read_Front (TIM_HandleTypeDef *htim)
{
	HAL_GPIO_WritePin(FRONT_TRIG_GPIO_Port, FRONT_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(FRONT_TRIG_GPIO_Port, FRONT_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC2);

}

void HCSR04_Read_Side (TIM_HandleTypeDef *htim)
{
	HAL_GPIO_WritePin(SIDE_TRIG_GPIO_Port, SIDE_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(SIDE_TRIG_GPIO_Port, SIDE_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC3);
}

double get_front_distance (void)
{
	return Front_US.DISTANCE;
}

double get_side_distance (void)
{
	return Side_US.DISTANCE;
}

double filter (double Sv, double old_Kv)
{
	double Kv = Sv*r + old_Kv*(1-r);
	return Kv;
}

