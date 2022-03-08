#include "imu.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f103xb.h"
#include "main.h"

// Define gyroscope and accelerometer registers
uint8_t ICM20948_ADDRESS = 0x68;
uint8_t REG_BANK_SEL = 0x7F;
uint8_t B0_GYRO_XOUT_H = 0x33;
uint8_t B0_ACCEL_XOUT_H = 0x2D;
uint8_t GYRO_CONFIG_1 = 0x01;
uint8_t ACCEL_CONFIG = 0x14;
uint8_t ICM20948_WHO_AM_I = 0x00;

uint8_t PWR_MGMT_2 = 0x07;

// Define magnetometer registers
uint8_t AK09916_ADDR = 0x0C;
uint8_t MAG_HXL = 0x11;

void Write_8(I2C_HandleTypeDef *hi2c1, uint8_t register_address,uint8_t data)
{
	uint8_t Trans[2]={register_address, data};
	HAL_I2C_Master_Transmit(hi2c1,ICM20948_ADDRESS << 1,Trans,2, 1000);
}

uint8_t Read_8(I2C_HandleTypeDef *hi2c1,uint8_t register_address){
	uint8_t Trans[1]={register_address};
	uint8_t Receive[1];
	HAL_I2C_Master_Transmit(hi2c1,ICM20948_ADDRESS << 1,Trans,1,1000);
	HAL_I2C_Master_Receive(hi2c1,ICM20948_ADDRESS << 1,Receive,1,1000);
	return Receive[0];
}

void Select_Bank(userbank ub, I2C_HandleTypeDef *hi2c1)
{
	Write_8(hi2c1, REG_BANK_SEL, ub);
}

uint8_t ICM_who_am_i(I2C_HandleTypeDef *hi2c1){
	Select_Bank(0, hi2c1);
	// should return 0xEA
	return Read_8(hi2c1,ICM20948_WHO_AM_I);
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

	//	Select_Bank(0, hi2c1);
	//	Write_8(hi2c1, PWR_MGMT_2, 0x80);
	//	Write_8(hi2c1, PWR_MGMT_2, 0x09);

	Select_Bank(2, hi2c1);
	Write_8(hi2c1, GYRO_CONFIG_1, 0x03); // set gyro range
	Write_8(hi2c1, ACCEL_CONFIG, 0x03); // set accel range

	for(int i=0; i<50; i++){
		axises accRawVal = ICM20948_Read_Accel(hi2c1);
		accel_bias.x += accRawVal.x;
		accel_bias.y += accRawVal.y;
		accel_bias.z += accRawVal.z;
		HAL_Delay(10);
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
