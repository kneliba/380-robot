/*
 * right_motor_encoder.h
 *
 *  Created on: Mar 5, 2022
 *      Author: kamilaneliba
 */

#ifndef INC_RIGHT_MOTOR_ENCODER_H_
#define INC_RIGHT_MOTOR_ENCODER_H_
#include "main.h"


typedef struct
{
	uint32_t	counter;
	int16_t 	count;
	int16_t 	position;
	double 		speed;
	double 		velocity;
	double 		distance;
	double 		wheel_radius;
}Motor_Encoder;

Motor_Encoder right_encoder;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void reset_distance();
int16_t get_distance_travelled();
int get_velocity();
int16_t get_encoder_count();

#endif /* INC_RIGHT_MOTOR_ENCODER_H_ */
