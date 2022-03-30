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
void drive_forward(TIM_HandleTypeDef *htim, double speed);

void drive_straight(TIM_HandleTypeDef *htim, double speed, I2C_HandleTypeDef *hi2c2, float desired_angle, float current_angle);

void drive_straight_PID(TIM_HandleTypeDef *htim, double speed, I2C_HandleTypeDef *hi2c2, float desired_angle, float current_angle, uint16_t dt);

void reset_PID_controller();

// Helper Constrain function
uint16_t constrain_value(uint16_t input, uint16_t min_val, uint16_t max_val);

// stop
void stop(TIM_HandleTypeDef *htim);

// turn right
void turn_right(TIM_HandleTypeDef *htim, double speed);
void turn_degree(TIM_HandleTypeDef *htim, I2C_HandleTypeDef *hi2c2, double angle);

// accelerate
void accelerate(TIM_HandleTypeDef *htim, double speed);

// decelerate
void decelerate(TIM_HandleTypeDef *htim);

// drive a set distance (with encoder)
void drive_distance(TIM_HandleTypeDef *htim, double speed, double distance);

// drive until a distance (with ultrasonic)
void drive_until(TIM_HandleTypeDef *htim, double speed, double distance);

void drive_straight_distance_ultrasonic(TIM_HandleTypeDef *htim, double speed, double distance, double ideal_block_distance);

void drive_straight_ultrasonic(TIM_HandleTypeDef *htim, double speed, double ideal_block_distance);
#endif /* ESC_H_ */
