//#include "main.h"
//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_tim.h"
//#include "stm32f1xx_hal_gpio.h"
//#include "right_motor_encoder.h"
//
//uint32_t counter = 0;
//int16_t count = 0;
//int16_t position = 0;
//int speed = 0;
//
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	counter = __HAL_TIM_GET_COUNTER(htim);
//	//count becomes negative rather than jumping to 65000
//	count = (int16_t)counter;
//	//a single count normally is counted by 4 points, will have to test the number
//	position = count/4;
//
//}
