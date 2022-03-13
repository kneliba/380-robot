#include "ESC.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"

static double ARR = 40000.0;

// drive forward - speed %
void drive_forward (TIM_HandleTypeDef *htim, double speed)
{
	double pulse_width = 1.0 + (speed/100.0);
	double command = (pulse_width/20.0)*ARR;

	TIM2->CCR1 = command; // left
	TIM2->CCR2 = command; // right
}

void stop (TIM_HandleTypeDef *htim)
{
	drive_forward (htim, 0);
}

// turn right
void turn_right (TIM_HandleTypeDef *htim)
{
	// spin left motor
	double speed = 10;
	double pulse_width = 1.0 + (speed/100.0);
	double command = (pulse_width/20.0)*ARR;
	TIM2->CCR1 = command;

	// hold right motor
	pulse_width = 1.0;
	command = (pulse_width/20.0)*ARR;
	TIM2->CCR2 = command;
}

// accelerate to desired speed
void accelerate (TIM_HandleTypeDef *htim, double final_speed)
{
	double speed = (((TIM2->CCR1)/ARR)*20.0 - 1)*100;
	while (speed < final_speed)
	{
		drive_forward(htim, speed);
		speed += 2;
		HAL_Delay(10);
	}
}

// decelerate to 0
void decelerate (TIM_HandleTypeDef *htim)
{
	// get current speed
	double speed = (((TIM2->CCR1)/ARR)*20.0 - 1)*100;
	while (speed > 0)
	{
		drive_forward(htim, speed);
		speed -= 2;
		HAL_Delay(10);
	}
}

// drive a set distance (with encoder)
void drive_distance (TIM_HandleTypeDef *htim, double speed, double distance)
{

}

// drive until a distance (with ultrasonic)
void drive_until (TIM_HandleTypeDef *htim, double speed, double distance)
{

}
