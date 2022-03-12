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
 *
 * PSC = 36-1
 * ARR = 40000-1
 * F_clk = 72MHz
 * F_pwm = 50Hz (20ms)
 *
 */

#ifndef ESC_H_
#define ESC_H_
#include "main.h"

// drive forward
void drive_forward (TIM_HandleTypeDef *htim, uint32_t speed);

// stop
void stop (TIM_HandleTypeDef *htim);

// turn right
//void turn_right (TIM_HandleTypeDef *htim);

#endif /* ESC_H_ */
