#include "esp.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f103xb.h"
#include "main.h"

void ESP_send(I2C_HandleTypeDef *hi2c1, uint8_t register_address, uint16_t size, uint8_t data)
{
	 uint8_t Trans[2]={register_address, data};
	 HAL_I2C_Slave_Transmit(hi2c1,Trans,size,1000);
}

void ESP_receive(I2C_HandleTypeDef *hi2c1, uint16_t size, uint8_t *Receive)
{
	HAL_I2C_Slave_Receive(hi2c1,Receive,size,1000);
}
