/*
 * tim_capture_callback.c
 *
 *  Created on: Mar 18, 2022
 *      Author: jcwon
 */
#include "right_motor_encoder.h"
#include "ultrasonic.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "main.h"

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1) //motor encoder
	{
		Motor_Encoder *right_motor_encoder;
		right_motor_encoder= &right_encoder;

		//CW is positive
		counter = __HAL_TIM_GET_COUNTER(htim);
		right_motor_encoder->counter = counter;

		//count becomes negative rather than jumping to 65000
		count = (int16_t)counter;
		right_motor_encoder->count = count;

		//a single count normally is counted by 4 points, will have to test the number
		position = count/4;
		right_motor_encoder->position = position;

		distance = (2*3.1415*right_motor_encoder->wheel_radius) * position; // might have consider gear ratio in this calculation
		right_motor_encoder->distance = distance;
	}

	else if (htim->Instance == TIM3) //ultrasonic
	{
		HCSR04_Type *ultrasonic;
		double sensor_val;

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // Front Trig
		{
			ultrasonic = &Front_US;
		}

		else if (TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) // Side Trig
		{
			ultrasonic = &Side_US;
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

}
