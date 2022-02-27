#include "imu.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f103xb.h"
#include "main.h"

// void ICM20948_Read_Gyro();

// void ICM20948_Read_Gyro();

void ICM20948_Read_Magn()
{
	HAL_StatusTypeDef ret;
	axises* data;
	uint8_t temp[6];

	// send write request to AK09916
    ret = HAL_I2C_Master_Transmit(&hi2c1, AK09916_ADDR << 1, &MAG_HXL, 1, 1000);

    if(ret != HAL_OK)
    {
    	return;
    }

	HAL_Delay(1);

	// read data starting from lower x-axis
	HAL_I2C_Master_Receive(&hi2c1, AK09916_ADDR << 1, temp, 6, 1000);

	// separate data into axises
    data->x = (int16_t)(temp[1] << 8 | temp[0]);
	data->y = (int16_t)(temp[3] << 8 | temp[2]);
	data->z = (int16_t)(temp[5] << 8 | temp[4]);

	// do something with data, convert to heading
}

// need function to select user bank ?
