#include "ESC.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
#include <imu.h>
#include <MadgwickAHRS.h>
//#include "ultrasonic.h"
//#include "right_motor_encoder.h"

static double ARR = 40000.0;

// drive forward - speed %
void drive_forward (TIM_HandleTypeDef *htim, double speed)
{
	double pulse_width = 1.0 + (speed/100.0);
	double command = (pulse_width/20.0)*ARR;

	TIM2->CCR1 = command; // left
	TIM2->CCR2 = command; // right
}

// drive forward - speed %
void drive_straight (TIM_HandleTypeDef *htim, double speed, I2C_HandleTypeDef *hi2c2)
{
	uint8_t correction_factor = 5;

	double pulse_width = 1.0 + (speed/100.0);
	double command = (pulse_width/20.0)*ARR;

	// Select User Bank 0
	ICM_SelectBank(hi2c2, USER_BANK_0);
	HAL_Delay(5);

	// Obtain raw accelerometer and gyro data
	ICM_ReadAccelGyro(hi2c2);

	// Obtain raw magnetometer data
	int16_t mag_data[3];
	ICM_ReadMag(hi2c2, mag_data);

	// Obtain corrected accelerometer and gyro data
	ICM_CorrectAccelGyro(hi2c2, accel_data, gyro_data);

	// Apply Madgwick to get pitch, roll, and yaw
	MadgwickAHRSupdate(corr_gyro_data[0], corr_gyro_data[1], corr_gyro_data[2],
					 corr_accel_data[0], corr_accel_data[1], corr_accel_data[2],
					 mag_data[0], mag_data[1], mag_data[2]);

	computeAngles();

	float roll_main = roll;
	float pitch_main = pitch;
	float yaw_main = yaw;

	float error = 0 - yaw_main;

	if (error < 0){
		// Correct by turning left
		TIM2->CCR1 = command -= error*correction_factor; // left
		TIM2->CCR2 = command += error*correction_factor; // right
	}

	else{
		// Correct by turning right
		TIM2->CCR1 = command += error*correction_factor; // left
		TIM2->CCR2 = command -= error*correction_factor; // right
	}



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

/* Integration functions

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
*/

void drive_straight_distance (TIM_HandleTypeDef *htim, double speed, double distance, I2C_HandleTypeDef *hi2c2)
{
	reset_distance();
	int16_t  encoder_dist = get_distance_travelled();
	drive_straight(htim, speed, hi2c2);
	while (encoder_dist < distance) {
		drive_straight(htim, speed, hi2c2);
	}
	stop(htim);
}
