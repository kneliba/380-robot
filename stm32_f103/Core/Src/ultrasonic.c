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

//void HCSR04_Init (void)
//{
//	Front_US.TRIG_GPIO = FRONT_TRIG_GPIO_Port;
//	Front_US.TRIG_PIN = FRONT_TRIG_Pin;
//	Front_US.TIM_Instance = TIM3;
//	Front_US.IC_TIM_CH = TIM_CHANNEL_2;
//	Front_US.VAL1 = 0;
//	Front_US.VAL2 = 0;
//	Front_US.DIFFERENCE = 0;
//	Front_US.FIRST_CAPTURED = 0;
//	Front_US.DISTANCE = 0;
//
//	Side_US.TRIG_GPIO = SIDE_TRIG_GPIO_Port;
//	Side_US.TRIG_PIN = SIDE_TRIG_Pin;
//	Side_US.TIM_Instance = TIM3;
//	Side_US.IC_TIM_CH = TIM_CHANNEL_3;
//	Side_US.VAL1 = 0;
//	Side_US.VAL2 = 0;
//	Side_US.DIFFERENCE = 0;
//	Side_US.FIRST_CAPTURED = 0;
//	Side_US.DISTANCE = 0;
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	HCSR04_Type *ultrasonic;
	double sensor_val;
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

				// Filter sensor data
				sensor_val = ultrasonic->DIFFERENCE * .034/2;
				ultrasonic->DISTANCE = filter(sensor_val, ultrasonic->DISTANCE);

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

