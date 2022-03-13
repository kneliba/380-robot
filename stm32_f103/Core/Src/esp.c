#include "esp.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f103xb.h"
#include "main.h"

uint8_t ESP_ADDRESS = 0x08; // not sure if correct, may have to check for address first

void ESP_send(I2C_HandleTypeDef *hi2c1, uint8_t register_address, uint16_t size, uint8_t data)
{
	 uint8_t Trans[2]={register_address, data};
	 HAL_I2C_Master_Transmit(hi2c1,ESP_ADDRESS << 1,Trans,size,1000);
}

void ESP_receive(I2C_HandleTypeDef *hi2c1, uint8_t register_address, uint16_t size, uint8_t *Receive)
{
	HAL_I2C_Master_Transmit(hi2c1,ESP_ADDRESS << 1,&register_address,1,1000);
	HAL_I2C_Master_Receive(hi2c1,ESP_ADDRESS << 1,Receive,size,1000);
}
