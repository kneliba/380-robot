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
void drive_forward (double speed);

void drive_straight_PID (double speed, I2C_HandleTypeDef *hi2c2, float current_angle, uint16_t dt);

void drive_straight_ultrasonic_P(TIM_HandleTypeDef *htim, double speed, double ideal_block_distance);

void drive_straight_ultrasonic_IMU(TIM_HandleTypeDef *htim, I2C_HandleTypeDef *hi2c2, double speed, double ideal_block_distance, double current_angle);

void reset_PID_controller();

// Helper Constrain function
uint16_t constrain_value(uint16_t input, uint16_t min_val, uint16_t max_val);

// stop
void stop ();

// drive distances
void drive_until (TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed, double distance);

// turn right
void turn_right (double speed);
void turn_degree (I2C_HandleTypeDef *hi2c2, double angle);

// accelerate
void accelerate (I2C_HandleTypeDef *hi2c2, double final_speed);

// decelerate
void decelerate (I2C_HandleTypeDef *hi2c2);
void adapt_decel (TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed, double distance);

void set_P_control_Kp (uint8_t P_Kp_value);
void set_PID_Kp (float PID_Kp_value);
void set_PID_Ki (float PID_Ki_value);
void set_PID_Kd (float PID_Kd_value);
void set_L_offset (double L_offset_value);
void set_min_dist (float min_dist_value);

uint8_t get_P_control_Kp ();
float get_PID_Kp ();
float get_PID_Ki ();
float get_PID_Kd ();
double get_L_offset ();
float get_min_dist ();

void help_im_stuck_stepbro();

void drive_pit (TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed, double ideal_block_distance);

#endif /* ESC_H_ */
