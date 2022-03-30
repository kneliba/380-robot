#include "ESC.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
//#include "ultrasonic.h"
//#include "right_motor_encoder.h"

static double ARR = 40000.0;
static double L_offset = 1.0;

static float integration_sum = 0;
static float prev_error = 0;

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
    static const float Kp = 10;
    static const float Ki = 0.0;
    static const float Kd = 2.5;

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

void reset_PID_controller(){
	integration_sum = 0;
	prev_error = 0;
}

void stop (TIM_HandleTypeDef *htim)
{
	drive_forward (htim, 0);
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
	TIM2->CCR2 = command;
}

void turn_degree (TIM_HandleTypeDef *htim, I2C_HandleTypeDef *hi2c2, double angle)
{
	uint16_t tick_rate = HAL_GetTickFreq();
	uint32_t last_tick = HAL_GetTick();
	double speed = 15;
	double curr_angle = 0;
	float dt = 0;
	ICM_SelectBank(hi2c2, USER_BANK_0);
	HAL_Delay(1);
	double error = curr_angle - (-angle);
	while (error > 5) {
		turn_right(htim, speed);
		ICM_ReadAccelGyro(hi2c2);
		ICM_CorrectAccelGyro(hi2c2, accel_data, gyro_data);
		dt = (float)(HAL_GetTick() -last_tick)/tick_rate;
		curr_angle += gyro_yaw(hi2c2, dt);
		last_tick = HAL_GetTick();
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
void decelerate (TIM_HandleTypeDef *htim)
{
	// get current speed
	double speed = (((TIM2->CCR1)/ARR)*20.0 - 1)*100;
	while (speed > 0)
	{
		drive_forward(htim, speed);
		speed -= 1;
		HAL_Delay(20);
	}
}


