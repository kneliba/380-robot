#include "ESC.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
#include <imu.h>
#include <stdbool.h>
//#include "ultrasonic.h"
//#include "right_motor_encoder.h"

static double speed = 20;

static double ARR = 40000.0;
static double L_offset = 1.5;

static float imu_integration_sum = 0;
static float imu_prev_error = 0;

static float us_integration_sum = 0;
static float us_prev_error = 0;

static float min_dist = 50; // distance to slow down [cm]

static float desired_angle = 0;

static uint8_t P_control_Kp = 8; // AMOG US PP
static float PID_Kp = 9.5;
static float PID_Ki = 0.012;
static float PID_Kd = 0.08;

static uint8_t stopping_dist = 12;

uint16_t constrain_value(uint16_t input, uint16_t min_val, uint16_t max_val){
	if (input < min_val) return min_val;
	else if (input > max_val) return max_val;
	else return input;
}

// drive forward - speed %
void drive_forward (double speed)
{
	double pulse_widthL = 1.0 + (speed*L_offset/100.0);
	double commandL = (pulse_widthL/20.0)*ARR;

	double pulse_widthR = 1.0 + (speed/100.0);
	double commandR = (pulse_widthR/20.0)*ARR;

	TIM2->CCR1 = commandL; // left
	TIM2->CCR2 = commandR; // right
}

void drive_until (TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed, double distance, double side_dist)
{
	uint8_t drive_speed = 12;

	reset_PID_controller();
	HCSR04_Read_Front(htim3);
	double ultrasonic_dist = get_front_distance();
	HAL_Delay(16);
	HCSR04_Read_Front(htim3);
	ultrasonic_dist = get_front_distance();
	double error = ultrasonic_dist - distance;
	if (error > min_dist) {
		accelerate(htim3, hi2c2, speed, side_dist);
	}

//	decelerate(hi2c2, drive_speed);
//	while (error > min_dist) {
	while (error > distance + stopping_dist) {
		get_imu_data(hi2c2);
		// Go slow to win uWu
//		drive_straight_PID(drive_speed, hi2c2, curr_pose.yaw, curr_pose.dt);
		drive_straight_path(htim3, hi2c2, drive_speed, side_dist);
		HCSR04_Read_Front(htim3);
		ultrasonic_dist = get_front_distance();
		error = ultrasonic_dist - distance;
	}

//	while (error > distance + 5){ // 5 for stopping dist @ 10 speed
//		get_imu_data(hi2c2);
//		drive_straight_PID(10, hi2c2, curr_pose.yaw, curr_pose.dt);
//		HCSR04_Read_Front(htim3);
//		ultrasonic_dist = get_front_distance();
//		error = ultrasonic_dist - distance;
//	}
//	adapt_decel(htim3, hi2c2, speed, distance);
	stop();
//	turn_degree(hi2c2, 90);
}

void drive_straight_PID (double speed, I2C_HandleTypeDef *hi2c2, float current_angle, uint16_t dt)
{
	static float Kp, Ki, Kd;
//	Kp = 10.2;
//	Ki = 0;
//	Kd = 0.08;

//
	Kp = PID_Kp;
    Ki = PID_Ki;
    Kd = PID_Kd;

//    current_angle = (int)current_angle%360;

    // Offset for each side due to drivetrain differences

    // TODO: Look into removing this float logic and change to integer logic with larger scale
    double pulse_widthL = 1.0 + (speed*L_offset/100.0);
    double pulse_widthR = 1.0 + (speed/100.0);

    uint16_t commandL = (pulse_widthL/20.0)*ARR;
    uint16_t commandR = (pulse_widthR/20.0)*ARR;

    float current_error = current_angle - desired_angle;

    // This is the PID controller calcs
    imu_integration_sum += (current_error * (dt/1000.0));
    float correction = Kp * current_error + Ki * imu_integration_sum + Kd * (current_error - imu_prev_error)/(dt/1000.0);
    imu_prev_error = current_error;

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

void drive_straight_ultrasonic (double speed, double ideal_block_distance)
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

void drive_straight_path(TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed, double ideal_block_distance)
{
	static const float Kp = 0.18;
	static const float Ki = 0.0;
	static const float Kd = 0.0;
	HCSR04_Read_Side(htim3);
	double distance_from_wall = get_side_distance();

	// Too far from wall = +ve error
	// Too close to wall = -ve error
	float current_error = distance_from_wall - ideal_block_distance;
	get_imu_data(hi2c2); // calculate yaw

	// This is the PID controller calcs
	us_integration_sum += (current_error * (curr_pose.dt/1000.0));
	float correction = Kp * current_error + Ki * us_integration_sum + Kd * (current_error - us_prev_error)/(curr_pose.dt/1000.0);
	us_prev_error = current_error;

//	if (current_error < 0){ // too close to wall
//		desired_angle -= correction;
//	}
//	else if (current_error > 0){ // too far from wall
//		desired_angle += correction;
//	}

	desired_angle = correction;
	drive_straight_PID(speed, hi2c2, curr_pose.yaw, curr_pose.dt);
}

void reset_PID_controller(){
	imu_integration_sum = 0;
	imu_prev_error = 0;
	us_integration_sum = 0;
	us_prev_error = 0;
	desired_angle = 0;
}

void stop ()
{
	drive_forward (0);
}

// drive until a distance (with ultrasonic)
// turn right
void turn_right (double speed)
{
	double pulse_width = 1.0 + (speed/100.0);
	double command = (pulse_width/20.0)*ARR;
	TIM2->CCR1 = command;

	// hold right motor
	pulse_width = 1.0;
	command = (pulse_width/20.0)*ARR;
	TIM2->CCR2 = constrain_value(command, 2000, 4000);
}

void turn_degree (I2C_HandleTypeDef *hi2c2, double angle)
{
	IMU_Init();
	reset_PID_controller();
	static float Kp, Ki, Kd;

	Kp = 0.38;
	Ki = 0.0;
	Kd = 0.005;

//	Kp = PID_Kp;
//	Ki = PID_Ki;
//	Kd = PID_Kd;

	static const uint8_t threshold = 15; // Acceptable angular setpoint error in degrees

	get_imu_data(hi2c2);
	double curr_angle = 0;

	// FIXME: Does this need to be here?
	ICM_SelectBank(hi2c2, USER_BANK_0);
	HAL_Delay(1);

	float current_error = curr_angle - (-angle);

	// This is the PID controller calcs
	imu_integration_sum += (current_error * (curr_pose.dt/1000.0));
	float correction = Kp * current_error + Ki * imu_integration_sum + Kd * (current_error - imu_prev_error)/(curr_pose.dt/1000.0);
	imu_prev_error = current_error;
	// Only Turning Right for now, dw about left

	while (current_error > threshold) {
		correction = constrain_value(correction, 0, 30);
		turn_right(correction);
		get_imu_data(hi2c2);

		current_error = curr_pose.yaw - (-angle);

		// This is the PID controller calcs
		imu_integration_sum += (current_error * (curr_pose.dt/1000.0));
		correction = Kp * current_error + Ki * imu_integration_sum + Kd * (current_error - imu_prev_error)/(curr_pose.dt/1000.0);
		imu_prev_error = current_error;
	}
	stop();
}

// accelerate to desired speed
void accelerate (TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double final_speed, double side_dist)
{
	double speed = (((TIM2->CCR1)/ARR)*20.0 - 1)*100;
	while (speed < final_speed)
	{
		get_imu_data(hi2c2);
//		drive_forward(speed);
		drive_straight_path(htim3, hi2c2, speed, side_dist);
//		drive_straight_PID(speed, hi2c2, curr_pose.yaw, curr_pose.dt);
		speed += 2;
//		HAL_Delay(10);
	}
}

// decelerate to 0
void decelerate (I2C_HandleTypeDef *hi2c2, uint8_t final_speed)
{
	// get current speed
	double speed = (((TIM2->CCR1)/ARR)*20.0 - 1)*100;
	while (speed > final_speed)
	{
		get_imu_data(hi2c2);
		drive_straight_PID(speed, hi2c2, curr_pose.yaw, curr_pose.dt);
//		drive_forward(speed);
		speed -= 1;
//		HAL_Delay(10);
	}
	stop();
}

void set_speed(double speed_value) {
	speed = speed_value;
}

void set_P_control_Kp (uint8_t P_Kp_value) {
	P_control_Kp = P_Kp_value;
}

void set_PID_Kp (float PID_Kp_value) {
	PID_Kp = PID_Kp_value;
}

void set_PID_Ki (float PID_Ki_value) {
	PID_Ki = PID_Ki_value;
}

void set_PID_Kd (float PID_Kp_value) {
	PID_Kp = PID_Kp_value;
}

void set_L_offset (double L_offset_value) {
	L_offset = L_offset_value;
}

void set_min_dist (float min_dist_value){
	min_dist = min_dist_value;
}

double get_speed() {
	return speed;
}

uint8_t get_P_control_Kp () {
	return P_control_Kp;
}

float get_PID_Kp () {
	return PID_Kp;
}

float get_PID_Ki () {
	return PID_Ki;
}

float get_PID_Kd () {
	return PID_Kd;
}

double get_L_offset () {
	return L_offset;
}

float get_min_dist () {
	return min_dist;
}


// decelerate relative to distance (ultrasonic)
void adapt_decel (TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed, double distance)
{
	float Kp = 15;
	HCSR04_Read_Front(htim3);
	float ultrasonic_dist = get_front_distance();
	float error = ultrasonic_dist-distance;
	//at a distance where we want to slow down
	while (error > 15 && speed > 5)
	{
		get_imu_data(hi2c2);
		speed = constrain_value(speed - (speed*Kp)/error, 0, speed);
		drive_straight_PID(speed, hi2c2, curr_pose.yaw, curr_pose.dt);
		HCSR04_Read_Front(htim3);
		ultrasonic_dist = get_front_distance();
	}
	stop();
}

void help_im_stuck_stepbro(){
	drive_forward(60);
	HAL_Delay(1000);
	stop();
}

void drive_pit (TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed, double ideal_block_distance) {
	bool pitched_down = false;
	bool pitched_up = false;
	float p_threshold = 10.0;
	reset_PID_controller(); // how often do we reset?
	// drive until reach pit
	while(!pitched_down)
	{
		get_imu_data(hi2c2);
		drive_straight_path(htim3, hi2c2, speed, ideal_block_distance);
		if(curr_pose.pitch < -p_threshold)
		{
			pitched_down = true;
		}
	}
	//speed up inside pit
	float pit_speed = speed;
	while(!pitched_up)
	{
		pit_speed += 2;
		get_imu_data(hi2c2);
		drive_straight_PID(constrain_value(pit_speed, speed, 50), hi2c2, curr_pose.yaw, curr_pose.dt); // TODO: may need to adjust
		if(curr_pose.pitch > p_threshold)
		{
			pitched_up = true;
		}
	}
	//still on an angle, finish climbing
	while(curr_pose.pitch > p_threshold)
	{
		get_imu_data(hi2c2);
		drive_straight_path(htim3, hi2c2, speed, ideal_block_distance);
	}
}
