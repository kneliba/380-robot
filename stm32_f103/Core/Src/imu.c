#include "imu.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f103xb.h"
#include "main.h"

void Select_Bank(userbank ub, I2C_HandleTypeDef *hi2c1)
{
	uint8_t Trans[2]={REG_BANK_SEL, ub};
	HAL_I2C_Master_Transmit(hi2c1,ICM20948_ADDRESS,Trans,2, 1000);
}


axises ICM20948_Read_Gyro(I2C_HandleTypeDef *hi2c1)
{
	HAL_StatusTypeDef ret;
	axises data;
	uint8_t temp[6];
	Select_Bank(0, hi2c1);

	// send write request to gyro
    ret = HAL_I2C_Master_Transmit(hi2c1, ICM20948_ADDRESS << 1, &B0_GYRO_XOUT_H, 1, 1000);

    if(ret != HAL_OK)
    {
    	// error message
    }

	HAL_Delay(1);

	// read data
	HAL_I2C_Master_Receive(hi2c1, ICM20948_ADDRESS << 1, temp, 6, 1000);

	data.x = (int16_t)(temp[0] << 8 | temp[1]);
	data.y = (int16_t)(temp[2] << 8 | temp[3]);
	data.z = (int16_t)(temp[4] << 8 | temp[5]);

	return data;
}

axises ICM20948_Read_Accel(I2C_HandleTypeDef *hi2c1)
{
	HAL_StatusTypeDef ret;
	axises data;
	uint8_t temp[6];
	Select_Bank(0, hi2c1);

	// send write request to accelerometer
    ret = HAL_I2C_Master_Transmit(hi2c1, ICM20948_ADDRESS << 1, &B0_ACCEL_XOUT_H, 1, 1000);

    if(ret != HAL_OK)
    {
    	// error message
    }

	HAL_Delay(1);

	// read data
	HAL_I2C_Master_Receive(hi2c1, ICM20948_ADDRESS << 1, temp, 6, 1000);

	data.x = (int16_t)(temp[0] << 8 | temp[1]);
	data.y = (int16_t)(temp[2] << 8 | temp[3]);
	data.z = (int16_t)(temp[4] << 8 | temp[5]);

	return data;
}

axises ICM20948_Read_Magn(I2C_HandleTypeDef *hi2c1)
{
	HAL_StatusTypeDef ret;
	axises data;
	uint8_t temp[6];
	Select_Bank(0, hi2c1);

	// send write request to AK09916
    ret = HAL_I2C_Master_Transmit(hi2c1, AK09916_ADDR << 1, &MAG_HXL, 1, 1000);

    if(ret != HAL_OK)
    {
    	// error message
    }

	HAL_Delay(1);

	// read data starting from lower x-axis
	HAL_I2C_Master_Receive(hi2c1, AK09916_ADDR << 1, temp, 6, 1000);

	// separate data into axises
    data.x = (int16_t)(temp[1] << 8 | temp[0]);
	data.y = (int16_t)(temp[3] << 8 | temp[2]);
	data.z = (int16_t)(temp[5] << 8 | temp[4]);

	return data;
}

void ICM20948_Calibrate(I2C_HandleTypeDef *hi2c1)
{
	 axises gyro_bias, accel_bias;

	Select_Bank(0, hi2c1);

	for(int i=0; i<50; i++){
		axises accRawVal = ICM20948_Read_Accel(hi2c1);
		accel_bias.x += accRawVal.x;
		accel_bias.y += accRawVal.y;
		accel_bias.z += accRawVal.z;
		HAL_Delay(1);
	}

	accel_bias.x /= 50;
	accel_bias.y /= 50;
	accel_bias.z /= 50;
	accel_bias.z -= 16384.0; // 16384 LSB/g

	for(int i=0; i<50; i++){
		axises gyrRawVal = ICM20948_Read_Gyro(hi2c1);
		gyro_bias.x += gyrRawVal.x;
		gyro_bias.y += gyrRawVal.y;
		gyro_bias.z += gyrRawVal.z;
		HAL_Delay(1);
	}

	gyro_bias.x /= 50;
	gyro_bias.y /= 50;
	gyro_bias.z /= 50;

	// set both offsets somewhere to use when reading accel and gyro values
}
