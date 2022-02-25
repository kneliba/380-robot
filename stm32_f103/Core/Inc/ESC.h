/*
 * ESC.h
 *
 * LEFT_PWM - TIM2_CH1
 * RIGHT_PWM - TIM2_CH2
 *
 * Commands:
 * 1ms -> full speed reverse
 * 2ms -> full speed forward
 * 1.5ms -> stop
 * ESC runs at 50Hz (20ms)
 *
 *  Created on: Feb. 25, 2022
 *      Author: jcwon
 */

#ifndef ESC_H_
#define ESC_H_
#include "main.h"

// drive forward
void drive_forward (uint32_t speed);

// stop
void stop (TIM_HandleTypeDef *htim);

// turn right
void turn_right (void);

// turn left
void turn_left (void);

#endif /* ESC_H_ */
