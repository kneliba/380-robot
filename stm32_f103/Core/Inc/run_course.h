/*
 * run_course.h
 *
 *  Created on: Mar. 30, 2022
 *      Author: junepark
 */

#ifndef INC_RUN_COURSE_H_
#define INC_RUN_COURSE_H_
#include "main.h"

// run full course by distance using ultrasonic only
void run_course_ultrasonic (TIM_HandleTypeDef *htim, double drive_speed, double turn_speed);

// run full course by distance using motor encoders only
void run_course_encoders (TIM_HandleTypeDef *htim, double drive_speed, double turn_speed);

// run full course by distance using motor encoders and ultrasonic
void run_course_encoders_ultrasonic (TIM_HandleTypeDef *htim, double drive_speed, double turn_speed);

#endif /* INC_RUN_COURSE_H_ */
