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
	HCSR04_Type *ultrasonic;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // Front Trig
	{
		ultrasonic = &Front_US;
	}

	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) // Side Trig
	{
		ultrasonic = &Side_US;
	}

	if (ultrasonic->FIRST_CAPTURED==0) // if the first value is not captured
	{
		ultrasonic->VAL1 = HAL_TIM_ReadCapturedValue(htim, ultrasonic->IC_TIM_CH); // read the first value
//			ultrasonic->VAL1 = __HAL_TIM_GET_COUNTER(htim);
		ultrasonic->FIRST_CAPTURED = 1;  // set the first captured as true
		// Now change the polarity to falling edge
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, ultrasonic->IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_FALLING);
	}

	else if (ultrasonic->FIRST_CAPTURED==1)   // if the first is already captured
	{
		ultrasonic->VAL2 = HAL_TIM_ReadCapturedValue(htim, ultrasonic->IC_TIM_CH);  // read second value
//			ultrasonic->VAL2 = __HAL_TIM_GET_COUNTER(htim);
		__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

		if (ultrasonic->VAL2 > ultrasonic->VAL1)
		{
			ultrasonic->DIFFERENCE = ultrasonic->VAL2-ultrasonic->VAL1;
		}

		else if (ultrasonic->VAL1 > ultrasonic->VAL2)
		{
			ultrasonic->DIFFERENCE = (0xffff - ultrasonic->VAL1) + ultrasonic->VAL2;
		}

		// Filter sensor data
		ultrasonic->SENSOR_VAL = ultrasonic->DIFFERENCE * .034/2;
//			ultrasonic->DISTANCE = filter(ultrasonic->SENSOR_VAL, ultrasonic->DISTANCE);
		ultrasonic->DISTANCE = ultrasonic->SENSOR_VAL;

		ultrasonic->FIRST_CAPTURED = 0; // set back to false

		// set polarity to rising edge
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, ultrasonic->IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_RISING);
		__HAL_TIM_DISABLE_IT(htim, ultrasonic->IC_TIM_CH);
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

float get_front_distance (void)
{
	return Front_US.DISTANCE;
}

float get_side_distance (void)
{
	return Side_US.DISTANCE;
}

float filter (double Sv, double old_Kv)
{
	double Kv = Sv*r + old_Kv*(1-r);
	return Kv;
}

