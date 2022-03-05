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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	HCSR04_Type *ultrasonic;
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // Front Trig
	{
		ultrasonic = &Front_US;
	}

	else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) // Side Trig
	{
		ultrasonic = &Side_US;
	}
	else
	{
		return;
	}
	if (ultrasonic->FIRST_CAPTURED==0) // if the first value is not captured
			{
				ultrasonic->VAL1 = HAL_TIM_ReadCapturedValue(htim, ultrasonic->IC_TIM_CH); // read the first value
				ultrasonic->FIRST_CAPTURED = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, ultrasonic->IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (ultrasonic->FIRST_CAPTURED==1)   // if the first is already captured
			{
				ultrasonic->VAL2 = HAL_TIM_ReadCapturedValue(htim, ultrasonic->IC_TIM_CH);  // read second value
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				if (ultrasonic->VAL2 > ultrasonic->VAL1)
				{
					ultrasonic->DIFFERENCE = ultrasonic->VAL2-ultrasonic->VAL1;
				}

				else if (ultrasonic->VAL1 > ultrasonic->VAL2)
				{
					ultrasonic->DIFFERENCE = (0xffff - ultrasonic->VAL1) + ultrasonic->VAL2;
				}

				ultrasonic->DISTANCE = ultrasonic->DIFFERENCE * .034/2;
				ultrasonic->FIRST_CAPTURED = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, ultrasonic->IC_TIM_CH, TIM_INPUTCHANNELPOLARITY_RISING);
				HAL_TIM_IC_Stop_IT(htim, ultrasonic->IC_TIM_CH);
			}
}

void HCSR04_Read_Front (TIM_HandleTypeDef *htim)
{
	HAL_GPIO_WritePin(FRONT_TRIG_GPIO_Port, FRONT_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(FRONT_TRIG_GPIO_Port, FRONT_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2); // enable interrupt on TIM3 CH2
}

void HCSR04_Read_Side (TIM_HandleTypeDef *htim)
{
	HAL_GPIO_WritePin(SIDE_TRIG_GPIO_Port, SIDE_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(SIDE_TRIG_GPIO_Port, SIDE_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_3); // enable interrupt on TIM3 CH3
}

