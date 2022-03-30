#include "ESC.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
#include "ultrasonic.h"
#include "right_motor_encoder.h"

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

// increases one side of the motor to drive faster to offset a leer
// can reduce or increase the tolerance from the ideal path and correction speed
// can also reduce the speed of the other motor to stay on path
void drive_straight_ultrasonic (TIM_HandleTypeDef *htim, double speed, double ideal_block_distance)
{

	double pulse_widthL = 1.0 + (speed*1.25/100.0);
	double commandL = (pulse_widthL/20.0)*ARR;

	double pulse_widthR = 1.0 + (speed/100.0);
	double commandR = (pulse_widthR/20.0)*ARR;

	double distance_from_wall = get_side_distance();
	int tolerance = 3;

	//float error = distance_from_wall - ideal_block_distance;

	uint8_t correction_speed = 15;
	double pulse_width_correction = 1.0 + ((correction_speed)/100.0);
	double increased_command = (pulse_width_correction/20.0)*ARR;


	// to the right of the ideal path
	if(distance_from_wall > ideal_block_distance + tolerance) {
		// Correct by driving to the left
		TIM2->CCR1 = commandL; // left
		TIM2->CCR2 = commandR + increased_command; // right
	}
	// to the left of the ideal path
	else if(distance_from_wall < ideal_block_distance - tolerance) {
		// Correct by driving to the right
		TIM2->CCR1 = commandL+ increased_command; // right
		TIM2->CCR2 = commandR; // right
	}
	// on ideal path
	else{
		// drive straight
		TIM2->CCR1 = commandL; // left
		TIM2->CCR2 = commandR; // right
	}
}

void drive_straight_distance_ultrasonic (TIM_HandleTypeDef *htim, double speed, double distance, double ideal_block_distance)
{
	reset_distance();
	int16_t  encoder_dist = get_distance_travelled();
	while (encoder_dist < distance) {
		drive_straight_ultrasonic(htim, speed, ideal_block_distance);
		encoder_dist = get_distance_travelled();
	}
	stop(htim);
}

// Integration functions
// drive a set distance (with encoder)
void drive_distance (TIM_HandleTypeDef *htim, double speed, double distance)
{
	reset_distance();
	int16_t  encoder_dist = get_distance_travelled();
	drive_forward(htim, speed);
	while (encoder_dist < distance) {}
	stop(htim);
}

// drive until a distance (with ultrasonic)
void drive_until (TIM_HandleTypeDef *htim, double speed, double distance)
{
	double ultrasonic_dist = get_front_distance();
	drive_foward(htim, speed);
	while (ultrasonic_dist < distance) {}
	stop(htim);
}
