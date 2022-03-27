/*
 * ESC.h
 *
 * LEFT_PWM - TIM2_CH1
 * RIGHT_PWM - TIM2_CH2
 *
 * Commands:
 * 1ms -> stop
 * 2ms -> full speed forward
 *
 * PSC = 4
 * ARR = 40000
 * F_clk = 8MHz
 * F_pwm = 50Hz (20ms)
 *
 */

#ifndef ESC_H_
#define ESC_H_
#include "main.h"

// drive forward
void drive_forward (TIM_HandleTypeDef *htim, double speed);

// stop
void stop (TIM_HandleTypeDef *htim);

// turn right
void turn_right (TIM_HandleTypeDef *htim);

// accelerate
void accelerate (TIM_HandleTypeDef *htim, double speed);

// decelerate
void decelerate (TIM_HandleTypeDef *htim);

// drive a set distance (with encoder)
void drive_distance (TIM_HandleTypeDef *htim, double speed, double distance);

// drive until a distance (with ultrasonic)
void drive_until (TIM_HandleTypeDef *htim, double speed, double distance);

#endif /* ESC_H_ */
