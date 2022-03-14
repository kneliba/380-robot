/*
 * ICM20948.c
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

// *** Three asterisks to the side of a line means this may change based on platform
#include "imu.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f103xb.h"
#include "main.h"
#include <string.h>

int16_t gyro_offset[3];
int16_t accel_offset[3];

void ICM_readBytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg | 0x80;
	uint8_t Trans[1]={reg};
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(hi2c,ICM20948_ADDRESS << 1,Trans,1,1000);
	ret = HAL_I2C_Master_Receive(hi2c,ICM20948_ADDRESS << 1,pData,Size,1000);
}

void ICM_WriteBytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg & 0x7F;
	uint8_t Trans[2]={reg, pData};
	HAL_I2C_Master_Transmit(hi2c,ICM20948_ADDRESS << 1,Trans,Size,1000);

}

void ICM_ReadOneByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t* pData) // ***
{
	reg = reg | 0x80;
	uint8_t Trans[1]={reg};
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Transmit(hi2c,ICM20948_ADDRESS << 1,Trans,1,1000);
	ret = HAL_I2C_Master_Receive(hi2c,ICM20948_ADDRESS << 1,pData,1,1000);
}

void ICM_WriteOneByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t Data) // ***
{
	reg = reg & 0x7F;
	uint8_t Trans[2]={reg, Data};
	HAL_I2C_Master_Transmit(hi2c,ICM20948_ADDRESS << 1,Trans,2, 1000);
}

/*
 *
 * AUX I2C abstraction for magnetometer
 *
 */
void i2c_Mag_write(I2C_HandleTypeDef *hi2c, uint8_t reg,uint8_t value)
  {
  	ICM_WriteOneByte(hi2c, 0x7F, 0x30);

  	HAL_Delay(1);
  	ICM_WriteOneByte(hi2c, 0x03 ,0x0C);//mode: write

  	HAL_Delay(1);
  	ICM_WriteOneByte(hi2c, 0x04 ,reg);//set reg addr

  	HAL_Delay(1);
  	ICM_WriteOneByte(hi2c, 0x06 ,value);//send value

  	HAL_Delay(1);
  }

  static uint8_t ICM_Mag_Read(I2C_HandleTypeDef *hi2c, uint8_t reg)
  {
  	uint8_t  Data;
  	ICM_WriteOneByte(hi2c, 0x7F, 0x30);
    HAL_Delay(1);
  	ICM_WriteOneByte(hi2c, 0x03 ,0x0C|0x80);
    HAL_Delay(1);
  	ICM_WriteOneByte(hi2c, 0x04 ,reg);// set reg addr
    HAL_Delay(1);
  	ICM_WriteOneByte(hi2c, 0x06 ,0xff);//read
  	HAL_Delay(1);
  	ICM_WriteOneByte(hi2c, 0x7F, 0x00);
  	ICM_ReadOneByte(hi2c, 0x3B,&Data);
    HAL_Delay(1);
  	return Data;
  }

  void ICM20948_READ_MAG(I2C_HandleTypeDef *hi2c, int16_t magn[3])
  {
    uint8_t mag_buffer[10];

      mag_buffer[0] =ICM_Mag_Read(hi2c, 0x01);
      mag_buffer[1] =ICM_Mag_Read(hi2c, 0x11);
  	  mag_buffer[2] =ICM_Mag_Read(hi2c, 0x12);
  	  magn[0]=mag_buffer[1]|mag_buffer[2]<<8;
      mag_buffer[3] =ICM_Mag_Read(hi2c, 0x13);
      mag_buffer[4] =ICM_Mag_Read(hi2c, 0x14);
      magn[1]=mag_buffer[3]|mag_buffer[4]<<8;
  	  mag_buffer[5] =ICM_Mag_Read(hi2c, 0x15);
      mag_buffer[6] =ICM_Mag_Read(hi2c, 0x16);
      magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

      i2c_Mag_write(hi2c, 0x31,0x01);
  }

/*
 *
 * Read magnetometer
 *
 */
void ICM_ReadMag(I2C_HandleTypeDef *hi2c, int16_t magn[3]) {
	uint8_t mag_buffer[10];
    mag_buffer[0] =ICM_Mag_Read(hi2c, 0x01);
    mag_buffer[1] =ICM_Mag_Read(hi2c, 0x11);
    mag_buffer[2] =ICM_Mag_Read(hi2c, 0x12);
    magn[0]=mag_buffer[1]|mag_buffer[2]<<8;
	mag_buffer[3] =ICM_Mag_Read(hi2c, 0x13);
    mag_buffer[4] =ICM_Mag_Read(hi2c, 0x14);
	magn[1]=mag_buffer[3]|mag_buffer[4]<<8;
	mag_buffer[5] =ICM_Mag_Read(hi2c, 0x15);
    mag_buffer[6] =ICM_Mag_Read(hi2c, 0x16);
	magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

	i2c_Mag_write(hi2c, 0x31,0x01);
}

/*
 *
 * Sequence to setup ICM290948 as early as possible after power on
 *
 */
void ICM_PowerOn(I2C_HandleTypeDef *hi2c) {
	// uint8_t test = ICM_WHOAMI(hi2c);
	HAL_Delay(10);
	ICM_SelectBank(hi2c, USER_BANK_0);
	HAL_Delay(10);
	ICM_Enable_I2C(hi2c);
	HAL_Delay(10);
	ICM_SetClock(hi2c, (uint8_t)CLK_BEST_AVAIL);
	HAL_Delay(10);
	ICM_AccelGyroOff(hi2c);
	HAL_Delay(20);
	ICM_AccelGyroOn(hi2c);
	HAL_Delay(10);
	ICM_Initialize(hi2c);
}

uint16_t ICM_Initialize(I2C_HandleTypeDef *hi2c) {
	ICM_SelectBank(hi2c, USER_BANK_2);
	HAL_Delay(20);
	ICM_SetGyroRateLPF(hi2c, GYRO_RATE_250, GYRO_LPF_17HZ);
	HAL_Delay(10);

	// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
	ICM_WriteOneByte(hi2c, 0x00, 0x0A);
	HAL_Delay(10);

	// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
	ICM_WriteOneByte(hi2c, 0x14, (0x04 | 0x11));

	// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
	ICM_WriteOneByte(hi2c, 0x10, 0x00);
	HAL_Delay(10);

	// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
	ICM_WriteOneByte(hi2c, 0x11, 0x0A);
	HAL_Delay(10);

	ICM_SelectBank(hi2c, USER_BANK_2);
	HAL_Delay(20);

	// Configure AUX_I2C Magnetometer (onboard ICM-20948)
	ICM_WriteOneByte(hi2c, 0x7F, 0x00); // Select user bank 0
	ICM_WriteOneByte(hi2c, 0x0F, 0x30); // INT Pin / Bypass Enable Configuration
	ICM_WriteOneByte(hi2c, 0x03, 0x20); // I2C_MST_EN
	ICM_WriteOneByte(hi2c, 0x7F, 0x30); // Select user bank 3
	ICM_WriteOneByte(hi2c, 0x01, 0x4D); // I2C Master mode and Speed 400 kHz
	ICM_WriteOneByte(hi2c, 0x02, 0x01); // I2C_SLV0 _DLY_ enable
	ICM_WriteOneByte(hi2c, 0x05, 0x81); // enable IIC	and EXT_SENS_DATA==1 Byte

	// Initialize magnetometer
	i2c_Mag_write(hi2c, 0x32, 0x01); // Reset AK8963
	HAL_Delay(1000);
	i2c_Mag_write(hi2c, 0x31, 0x02); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output

	return 1337;
}

void ICM_ReadAccelGyro(I2C_HandleTypeDef *hi2c) {
	uint8_t raw_data[12];
	ICM_readBytes(hi2c, 0x2D, raw_data, 12);

	accel_data[0] = (raw_data[0] << 8) | raw_data[1];
	accel_data[1] = (raw_data[2] << 8) | raw_data[3];
	accel_data[2] = (raw_data[4] << 8) | raw_data[5];

	gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
	gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
	gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

	accel_data[0] = accel_data[0] / 8;
	accel_data[1] = accel_data[1] / 8;
	accel_data[2] = accel_data[2] / 8;

	gyro_data[0] = gyro_data[0] / 250;
	gyro_data[1] = gyro_data[1] / 250;
	gyro_data[2] = gyro_data[2] / 250;
}

void ICM_CorrectAccelGyro(I2C_HandleTypeDef *hi2c, uint16_t raw_accel_data[3], uint16_t raw_gyro_data[3]) {
	corr_accel_data[0] = (raw_accel_data[0] - (accel_offset[0] / (1<<GYRO_RATE_250)));
	corr_accel_data[1] = (raw_accel_data[1] - (accel_offset[1] / (1<<GYRO_RATE_250)));
	corr_accel_data[2] = (raw_accel_data[2] - (accel_offset[2] / (1<<GYRO_RATE_250)));

	corr_gyro_data[0] = (raw_gyro_data[0] - (gyro_offset[0] / (1<<0x04)));
	corr_gyro_data[1] = (raw_gyro_data[0] - (gyro_offset[0] / (1<<0x04)));
	corr_gyro_data[2] = (raw_gyro_data[0] - (gyro_offset[0] / (1<<0x04)));
}

void ICM_SelectBank(I2C_HandleTypeDef *hi2c, uint8_t bank) {
	ICM_WriteOneByte(hi2c, USER_BANK_SEL, bank);
}

void ICM_Enable_I2C(I2C_HandleTypeDef *hi2c) {
	ICM_WriteOneByte(hi2c, 0x03, 0x20);
}

void ICM_SetClock(I2C_HandleTypeDef *hi2c, uint8_t clk) {
	ICM_WriteOneByte(hi2c, PWR_MGMT_1, clk);
}

void ICM_AccelGyroOff(I2C_HandleTypeDef *hi2c) {
	ICM_WriteOneByte(hi2c, PWR_MGMT_2, (0x38 | 0x07));
}

void ICM_AccelGyroOn(I2C_HandleTypeDef *hi2c) {
	ICM_WriteOneByte(hi2c, 0x07, (0x00 | 0x00));
}

uint8_t ICM_WHOAMI(I2C_HandleTypeDef *hi2c) {
	uint8_t i2cData = 0x01;
	ICM_ReadOneByte(hi2c, 0x00, &i2cData);
	return i2cData;
}

void ICM_SetGyroRateLPF(I2C_HandleTypeDef *hi2c, uint8_t rate, uint8_t lpf) {
	ICM_WriteOneByte(hi2c, GYRO_CONFIG_1, (rate|lpf));
}

void ICM20948_Calibrate(I2C_HandleTypeDef *hi2c)
{
	// Calibrate accelerometer
	for(int i=0; i<50; i++){
		ICM_ReadAccelGyro(hi2c);
		accel_offset[0] += accel_data[0];
		accel_offset[1] += accel_data[1];
		accel_offset[2] += accel_data[2];
		HAL_Delay(10);
	}

	accel_offset[0] /= 50;
	accel_offset[1] /= 50;
	accel_offset[2] /= 50;
	accel_offset[2] -= 16384.0; // 16384 LSB/g

	// Calibrate gyroscope
	for(int i=0; i<50; i++){
		ICM_ReadAccelGyro(hi2c);
		gyro_offset[0] += gyro_data[0];
		gyro_offset[1] += gyro_data[1];
		gyro_offset[2] += gyro_data[2];
		HAL_Delay(1);
	}

	gyro_offset[0] /= 50;
	gyro_offset[1] /= 50;
	gyro_offset[2] /= 50;
}

