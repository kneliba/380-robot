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
	0,
	0
};

void HCSR04_timer_input_CC (TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // Front Trig
	{
		if (Front_US.FIRST_CAPTURED==0) // if the first value is not captured
		{
			Front_US.VAL1 = HAL_TIM_ReadCapturedValue(htim, Front_US.IC_TIM_CH); // read the first value
	//			Front_US.VAL1 = __HAL_TIM_GET_COUNTER(htim);
			Front_US.FIRST_CAPTURED = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, Front_US.IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Front_US.FIRST_CAPTURED==1)   // if the first is already captured
		{
			Front_US.VAL2 = HAL_TIM_ReadCapturedValue(htim, Front_US.IC_TIM_CH);  // read second value
	//			Front_US.VAL2 = __HAL_TIM_GET_COUNTER(htim);
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			Front_US.FIRST_CAPTURED = 0; // set back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, Front_US.IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, Front_US.IC_TIM_CH);
		}
	}

	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) // Side Trig
	{
		if (Side_US.FIRST_CAPTURED==0) // if the first value is not captured
		{
			Side_US.VAL1 = HAL_TIM_ReadCapturedValue(htim, Side_US.IC_TIM_CH); // read the first value
	//			Side_US.VAL1 = __HAL_TIM_GET_COUNTER(htim);
			Side_US.FIRST_CAPTURED = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, Side_US.IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Side_US.FIRST_CAPTURED==1)   // if the first is already captured
		{
			Side_US.VAL2 = HAL_TIM_ReadCapturedValue(htim, Side_US.IC_TIM_CH);  // read second value
	//			Side_US.VAL2 = __HAL_TIM_GET_COUNTER(htim);
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			Side_US.FIRST_CAPTURED = 0; // set back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, Side_US.IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, Side_US.IC_TIM_CH);
		}
	}


}

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
	while (Front_US.FIRST_CAPTURED != 0) {} // wait until full pulse is recieved

	if (Front_US.VAL2 > Front_US.VAL1)
	{
		Front_US.DIFFERENCE = Front_US.VAL2-Front_US.VAL1;
	}

	else if (Front_US.VAL1 > Front_US.VAL2)
	{
		Front_US.DIFFERENCE = (0xffff - Front_US.VAL1) + Front_US.VAL2;
	}

	// Filter sensor data
	Front_US.SENSOR_VAL = Front_US.DIFFERENCE * .034/2;
//	Front_US.DISTANCE = filter(Front_US.SENSOR_VAL, Front_US.DISTANCE);
	return Front_US.SENSOR_VAL;

}

double get_side_distance (void)
{
	while (Side_US.FIRST_CAPTURED != 0) {}

	if (Side_US.VAL2 > Side_US.VAL1)
	{
		Side_US.DIFFERENCE = Side_US.VAL2-Side_US.VAL1;
	}

	else if (Side_US.VAL1 > Side_US.VAL2)
	{
		Side_US.DIFFERENCE = (0xffff - Side_US.VAL1) + Side_US.VAL2;
	}

	// Filter sensor data
	Side_US.SENSOR_VAL = Side_US.DIFFERENCE * .034/2;
//	Side_US.DISTANCE = filter(Side_US.SENSOR_VAL, Side_US.DISTANCE);
	return Side_US.SENSOR_VAL;
}

double filter (double Sv, double old_Kv)
{
	double Kv = Sv*r + old_Kv*(1-r);
	return Kv;
}

