/*
 * run_course.c
 *
 *  Created on: Mar. 25, 2022
 *      Author: junepark
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
#include "ESC.h"

// distances to wall to travel in cm
double distances_to_wall[11] = {15, 15, 15, 45.5, 45.5, 45.5, 45.5, 76, 76, 76, 76};

// run full course using ultrasonic only
void run_course_ultrasonic (TIM_HandleTypeDef *htim, double speed)
{
	int8_t current = 0;

	while(current < 11)
	{
		// error correction, use angle or side ultrasonic to fix angle
		drive_until(htim, speed, distances_to_wall[current]);
		turn_right(htim);
		current++;
	}
}
