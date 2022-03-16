#include "right_motor_encoder.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "main.h"


uint32_t counter = 0;
int16_t count = 0;
int16_t position = 0;
double speed = 0;
double velocity = 0;
double distance = 0;

Motor_Encoder right_encoder = {
		0,
		0,
		0,
		0,
		0,
		0,
		0.07042,
};

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		Motor_Encoder *right_motor_encoder;
		right_motor_encoder= &right_encoder;

		//CW is positive
		counter = __HAL_TIM_GET_COUNTER(htim);
		right_motor_encoder->counter = counter;

		//count becomes negative rather than jumping to 65000
		count = (int16_t)counter;
		right_motor_encoder->count = count;

		//a single count normally is counted by 4 points, will have to test the number
		position = count/4;
		right_motor_encoder->position = position;

		distance = (2*3.1415*right_motor_encoder->wheel_radius) * position; // might have consider gear ratio in this calculation
		right_motor_encoder->distance = distance;
	}
}

void reset_distance()
{
	Motor_Encoder *right_motor_encoder;
	right_motor_encoder= &right_encoder;
	right_motor_encoder->counter = 0;
	right_motor_encoder->count = 0;
	right_motor_encoder->position = 0;
	right_motor_encoder->distance = 0;
}

int16_t get_distance_travelled()
{
	Motor_Encoder *right_motor_encoder;
	right_motor_encoder= &right_encoder;
	return right_motor_encoder->position; // should be distance once wheel attached
}

int get_velocity()
{
	Motor_Encoder *right_motor_encoder;
	right_motor_encoder= &right_encoder;
	return right_motor_encoder->speed; //should be velocity once wheel is attached
}

int16_t get_encoder_count()
{
	Motor_Encoder *right_motor_encoder;
	right_motor_encoder= &right_encoder;
	return right_motor_encoder->count;
}

