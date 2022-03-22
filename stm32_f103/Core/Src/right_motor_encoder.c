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

void reset_distance(TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
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
	return right_motor_encoder->distance; // should be distance once wheel attached
}

int get_velocity()
{
	Motor_Encoder *right_motor_encoder;
	right_motor_encoder= &right_encoder;
	return right_motor_encoder->velocity; //should be velocity once wheel is attached
}

int16_t get_encoder_count()
{
	Motor_Encoder *right_motor_encoder;
	right_motor_encoder= &right_encoder;
	return right_motor_encoder->count;
}

