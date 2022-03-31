#include "ESC.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
//#include "ultrasonic.h"
//#include "right_motor_encoder.h"

static double ARR = 40000.0;
static double L_offset = 1.1;

static float integration_sum = 0;
static float prev_error = 0;

static double min_dist = 20; // distance to slow down [cm]

uint16_t constrain_value(uint16_t input, uint16_t min_val, uint16_t max_val){
	if (input < min_val) return min_val;
	else if (input > max_val) return max_val;
	else return input;
}

// drive forward - speed %
void drive_forward (TIM_HandleTypeDef *htim, double speed)
{
	double pulse_widthL = 1.0 + (speed*L_offset/100.0);
	double commandL = (pulse_widthL/20.0)*ARR;

	double pulse_widthR = 1.0 + (speed/100.0);
	double commandR = (pulse_widthR/20.0)*ARR;

	TIM2->CCR1 = commandL; // left
	TIM2->CCR2 = commandR; // right
}


void drive_straight (TIM_HandleTypeDef *htim, double speed, I2C_HandleTypeDef *hi2c2, float desired_angle, float current_angle)
{
    uint8_t kp = 8;

//    current_angle = (int)current_angle%360;

    double pulse_widthL = 1.0 + (speed*L_offset/100.0);
    double commandL = (pulse_widthL/20.0)*ARR;

    double pulse_widthR = 1.0 + (speed/100.0);
    double commandR = (pulse_widthR/20.0)*ARR;


    float error = current_angle - desired_angle;

    if (error < 0){
        // Correct by turning left
        double new_command = commandL + error*kp; //results in decrease bc negative error
        if (new_command < 2000)
        {
            TIM2->CCR1 = 2000;
            TIM2->CCR2 = commandR;
        }
        else
        {
            TIM2->CCR1 = new_command;
            TIM2->CCR2 = commandR;
        }
    }
    else if (error > 0){
        // Correct by turning right
        double new_command = commandR - error*kp;
        if (new_command < 2000)
        {
            TIM2->CCR2 = 2000;
            TIM2->CCR1 = commandL;
        }
        else
        {
            TIM2->CCR2 = new_command;
            TIM2->CCR1 = commandL;
        }
    }
    else
    {
    	TIM2->CCR1 = commandL;
    	TIM2->CCR2 = commandR;
    }
}

void drive_straight_PID (TIM_HandleTypeDef *htim, double speed, I2C_HandleTypeDef *hi2c2, float desired_angle, float current_angle, uint16_t dt)
{
    static const float Kp = 7;
    static const float Ki = 0.0;
    static const float Kd = 0;

//    current_angle = (int)current_angle%360;

    // Offset for each side due to drivetrain differences

    // TODO: Look into removing this float logic and change to integer logic with larger scale
    double pulse_widthL = 1.0 + (speed*L_offset/100.0);
    double pulse_widthR = 1.0 + (speed/100.0);

    uint16_t commandL = (pulse_widthL/20.0)*ARR;
    uint16_t commandR = (pulse_widthR/20.0)*ARR;

    float current_error = current_angle - desired_angle;

    // This is the PID controller calcs
    integration_sum += (current_error * (dt/1000.0));
    float correction = Kp * current_error + Ki * integration_sum + Kd * (current_error - prev_error)/(dt/1000.0);
    prev_error = current_error;

    if (current_error < 0){
        // Correct by turning left
        double new_command = commandL + correction; //results in decrease bc negative error
        TIM2->CCR1 = constrain_value(new_command, 2000, 4000);
		TIM2->CCR2 = commandR;
    }
    else if (current_error > 0){
        // Correct by turning right
        double new_command = commandR - correction;
        TIM2->CCR1 = commandL;
		TIM2->CCR2 = constrain_value(new_command, 2000, 4000);;
    }
    else
    {
    	TIM2->CCR1 = commandL;
    	TIM2->CCR2 = commandR;
    }
}

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

void drive_straight_distance_ultrasonic (TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, double speed, double distance, double ideal_block_distance)
{
	reset_distance(htim1);
	int16_t  encoder_dist = get_distance_travelled();
	while (encoder_dist < distance) {
		drive_straight_ultrasonic(htim3, speed, ideal_block_distance);
		encoder_dist = get_distance_travelled();
	}
	stop(htim2);
}

void reset_PID_controller(){
	integration_sum = 0;
	prev_error = 0;
}

void stop (TIM_HandleTypeDef *htim)
{
	drive_forward (htim, 0);
}

// drive a set distance (with encoder)
void drive_distance (TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, I2C_HandleTypeDef *hi2c2, double speed, double distance)
{
	reset_distance(htim1);
	double encoder_dist = get_distance_travelled();
	double yaw = 0;
	while (encoder_dist < distance) {
		get_imu_data(hi2c2);
		yaw += curr_pose.yaw;
		drive_straight(htim2, speed, hi2c2, 0, yaw);
	}
	stop(htim2);
}

// drive until a distance (with ultrasonic)
void drive_until (TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed, double distance)
{
	double ultrasonic_dist = 0;
	for (int i = 0; i<5; i++)
	{
		HCSR04_Read_Front(htim3);
		ultrasonic_dist = get_front_distance();
		HAL_Delay(30);
	}
	double yaw = 0;
	double error = ultrasonic_dist - distance;
//	drive_forward(htim2, speed);
	while (error > min_dist) {
		get_imu_data(hi2c2);
		yaw += curr_pose.yaw;
		drive_straight(htim2, speed, hi2c2, 0, yaw);
		HCSR04_Read_Front(htim3);
		ultrasonic_dist = get_front_distance();
		error = ultrasonic_dist - distance;
		HAL_Delay(15);
	}
	decelerate(htim2);
//	adapt_decel(htim2, htim3, hi2c2, speed, distance);
}

// turn right
void turn_right (TIM_HandleTypeDef *htim, double speed)
{
	double pulse_width = 1.0 + (speed/100.0);
	double command = (pulse_width/20.0)*ARR;
	TIM2->CCR1 = command;

	// hold right motor
	pulse_width = 1.0;
	command = (pulse_width/20.0)*ARR;
	TIM2->CCR2 = constrain_value(command, 2000, 4000);
}

void turn_degree (TIM_HandleTypeDef *htim, I2C_HandleTypeDef *hi2c2, double angle)
{
	reset_PID_controller();

	static const float Kp = 0.23;
	static const float Ki = 0.0;
	static const float Kd = 0.008;

	static const uint8_t threshold = 5; // Acceptable angular setpoint error in degrees

	get_imu_data(hi2c2);
	double curr_angle = 0;

	// FIXME: Does this need to be here?
	ICM_SelectBank(hi2c2, USER_BANK_0);
	HAL_Delay(1);

	float current_error = curr_angle - (-angle);

	// This is the PID controller calcs
	integration_sum += (current_error * (curr_pose.dt/1000.0));
	float correction = Kp * current_error + Ki * integration_sum + Kd * (current_error - prev_error)/(curr_pose.dt/1000.0);
	prev_error = current_error;
	// Only Turning Right for now, dw about left

	while (current_error > threshold) {
		turn_right(htim, correction);
		get_imu_data(hi2c2);
		curr_angle += curr_pose.yaw;

		current_error = curr_angle - (-angle);

		// This is the PID controller calcs
		integration_sum += (current_error * (curr_pose.dt/1000.0));
		correction = Kp * current_error + Ki * integration_sum + Kd * (current_error - prev_error)/(curr_pose.dt/1000.0);
		prev_error = current_error;

	}
	stop(htim);
}

// accelerate to desired speed
void accelerate (TIM_HandleTypeDef *htim, double final_speed)
{
	double speed = (((TIM2->CCR1)/ARR)*20.0 - 1)*100;
	while (speed < final_speed)
	{
		drive_forward(htim, speed);
		speed += 1;
		HAL_Delay(20);
	}
}

// decelerate to 0
void adapt_decel (TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed, double distance)
{
	HCSR04_Read_Front(htim3);
	double ultrasonic_dist = get_front_distance();
	//at a distance where we want to slow down
	while (speed > 10)
	{
		double error = ultrasonic_dist-distance;
		get_imu_data(hi2c2);
		yaw += curr_pose.yaw;
		speed = speed/(0.5/error);
		drive_straight(htim2, speed, hi2c2, 0, yaw);
		HCSR04_Read_Front(htim3);
		ultrasonic_dist = get_front_distance();
		HAL_Delay(20);
	}
	stop(htim2);
}


void decelerate (TIM_HandleTypeDef *htim)
{
	// get current speed
	double speed = (((TIM2->CCR1)/ARR)*20.0 - 1)*100;
	while (speed > 0)
	{
		drive_forward(htim, speed);
		speed -= 1;
		HAL_Delay(10);
	}
}

// decelerate relative to distance (ultrasonic)


// increases one side of the motor to drive faster to offset a leer
// can also reduce the speed of the other motor to stay on path
void drive_straight_ultrasonic_P(TIM_HandleTypeDef *htim, double speed, double ideal_block_distance)
{
	uint8_t kp = 8;
	double pulse_widthL = 1.0 + (speed * L_offset / 100.0);
	double pulse_widthR = 1.0 + (speed / 100.0);

	uint16_t commandL = (pulse_widthL / 20.0) * ARR;
	uint16_t commandR = (pulse_widthR / 20.0) * ARR;

	double distance_from_wall = get_side_distance();

	float error = distance_from_wall - ideal_block_distance;

	// to the left of the ideal path
	if (error < 0)
	{
		// Correct by driving to the right (negative error)
		double new_command = commandL + error * kp;
		if (new_command < 2000)
		{
			TIM2->CCR1 = commandL;
			TIM2->CCR2 = 2000;
		}
		else
		{
			TIM2->CCR1 = commandR;
			TIM2->CCR2 = new_command;
		}
	}
	// to the right of the ideal path
	else if (error > 0)
	{
		// Correct by driving to the right
		double new_command = commandL + error * kp;
		if (new_command < 2000)
		{
			TIM2->CCR1 = commandL;
			TIM2->CCR2 = 2000;
		}
		else
		{
			TIM2->CCR1 = commandR;
			TIM2->CCR2 = new_command;
		}
	}
	// on ideal path
	else
	{
		// drive straight
		TIM2->CCR1 = commandL; // left
		TIM2->CCR2 = commandR; // right
	}
}

// increases one side of the motor to drive faster to offset a leer
// can also reduce the speed of the other motor to stay on path
void drive_straight_ultrasonic_IMU(TIM_HandleTypeDef *htim, I2C_HandleTypeDef *hi2c2, double speed, double ideal_block_distance, double current_angle)
{
	uint8_t kp = 0.8;

	double distance_from_wall = get_side_distance();

	float distance_error = distance_from_wall - ideal_block_distance;
	float desired_angle = distance_error*kp*(-1)+current_angle;

	drive_straight(htim , speed, hi2c2, desired_angle, current_angle);
}
