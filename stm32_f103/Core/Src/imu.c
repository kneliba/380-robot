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

int16_t accel_data[3] = {};
int16_t gyro_data[3] = {};
int16_t mag_data[3] = {};

float corr_accel_data[3] = {};
float corr_gyro_data[3] = {};

int16_t gyro_offset[3];
int16_t accel_offset[3];

static uint16_t tick_rate;
static uint32_t last_tick;
static double dt = 0;

robot_pose curr_pose = {0, 0, 0, 0};

void ICM_WriteOneByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *pData) // ***
{
	reg = reg & 0x7F;
//	uint8_t Trans[2]={reg, Data};
//	HAL_I2C_Master_Transmit(hi2c,ICM20948_ADDRESS << 1,Trans,2, 1000);
	HAL_I2C_Mem_Write(hi2c, ICM20948_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, pData, 1, 1000);
}

void ICM_readBytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg | 0x80;
	uint8_t Trans[1]={reg};
	HAL_StatusTypeDef ret;
//	ret = HAL_I2C_Master_Transmit(hi2c,ICM20948_ADDRESS << 1,Trans,1,100);
//	ret = HAL_I2C_Master_Receive(hi2c,ICM20948_ADDRESS << 1,pData,Size,100);
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, pData, Size, 1000);

//	if (ret == HAL_BUSY){
//		// Reset I2C Master
//		ICM_WriteOneByte(hi2c, 0x7F, 0x00); // Select user bank 0
//		HAL_Delay(5);
//		ICM_WriteOneByte(hi2c, 0x03, 0xE); // reset everything maybe
//	}
}

// not used
void ICM_WriteBytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg & 0x7F;
	uint8_t Trans[2]={reg, pData};
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c,ICM20948_ADDRESS << 1,Trans,Size,100);
//	if (ret == HAL_BUSY){
//		// Reset I2C Master
//		ICM_WriteOneByte(hi2c, 0x7F, 0x00); // Select user bank 0
//		HAL_Delay(5);
//		ICM_WriteOneByte(hi2c, 0x03, 0xE); // reset everything maybe
//	}

}

void ICM_ReadOneByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t* pData) // ***
{
	reg = reg | 0x80;
	uint8_t Trans[1]={reg};
	HAL_StatusTypeDef ret;
//	ret = HAL_I2C_Master_Transmit(hi2c,ICM20948_ADDRESS << 1,Trans,1,100);
//	ret = HAL_I2C_Master_Receive(hi2c,ICM20948_ADDRESS << 1,pData,1,100);

	HAL_I2C_Mem_Read(hi2c, ICM20948_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, pData, 1, 1000);
//	if (ret == HAL_BUSY){
//		// Reset I2C Master
//		ICM_WriteOneByte(hi2c, 0x7F, 0x00); // Select user bank 0
//		HAL_Delay(5);
//		ICM_WriteOneByte(hi2c, 0x03, 0xE); // reset everything maybe
//	}
}

/*
 *
 * AUX I2C abstraction for magnetometer
 *
 */
void i2c_Mag_write(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
  {
	uint8_t writeData = 0x30;
  	ICM_WriteOneByte(hi2c, 0x7F, &writeData);
  	HAL_Delay(1);
  	writeData = 0x0C;
  	ICM_WriteOneByte(hi2c, 0x03, &writeData);//mode: write
  	HAL_Delay(1);
  	writeData = reg;
  	ICM_WriteOneByte(hi2c, 0x04, &writeData);//set reg addr
  	HAL_Delay(1);
  	writeData = value;
  	ICM_WriteOneByte(hi2c, 0x06, &writeData);//send value
  	HAL_Delay(1);
  }

static uint8_t ICM_Mag_Read(I2C_HandleTypeDef *hi2c, uint8_t reg)
  {
  	uint8_t  Data;
  	uint8_t writeData = 0x30;
  	ICM_WriteOneByte(hi2c, 0x7F, &writeData);
    HAL_Delay(1);
    writeData = (0x0C|0x80);
  	ICM_WriteOneByte(hi2c, 0x03, &writeData);
    HAL_Delay(1);
    writeData = reg;
  	ICM_WriteOneByte(hi2c, 0x04, &writeData);// set reg addr
    HAL_Delay(1);
    writeData = 0xff;
  	ICM_WriteOneByte(hi2c, 0x06, &writeData);//read
  	HAL_Delay(1);
  	writeData = 0x00;
  	ICM_WriteOneByte(hi2c, 0x7F, &writeData);
  	ICM_ReadOneByte(hi2c, 0x3B, &Data);
    HAL_Delay(1);
  	return Data;
}

/*
 *
 * Read magnetometer
 *
 */
void ICM_ReadMag(I2C_HandleTypeDef *hi2c, int16_t magn[3]) {
//	uint8_t mag_buffer[10];
//    mag_buffer[0] =ICM_Mag_Read(hi2c, 0x01);
//    mag_buffer[1] =ICM_Mag_Read(hi2c, 0x11);
//    mag_buffer[2] =ICM_Mag_Read(hi2c, 0x12);
//    magn[0]=mag_buffer[1]|mag_buffer[2]<<8;
//	mag_buffer[3] =ICM_Mag_Read(hi2c, 0x13);
//    mag_buffer[4] =ICM_Mag_Read(hi2c, 0x14);
//	magn[1]=mag_buffer[3]|mag_buffer[4]<<8;
//	mag_buffer[5] =ICM_Mag_Read(hi2c, 0x15);
//    mag_buffer[6] =ICM_Mag_Read(hi2c, 0x16);
//	magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

//	i2c_Mag_write(hi2c, 0x31,0x01); why is it in single measurement mode

	uint8_t mag_buffer[12];
	HAL_I2C_Mem_Read(hi2c, AK09916_ADDRESS << 1, 0x11, I2C_MEMADD_SIZE_8BIT, mag_buffer, 12, 1000);

	magn[0] = (int16_t)(mag_buffer[0] | mag_buffer[1] << 8);
	magn[1] = (int16_t)(mag_buffer[2] | mag_buffer[3] << 8);
	magn[2] = (int16_t)(mag_buffer[4] | mag_buffer[5] << 8);

}

/*
 *
 * Sequence to setup ICM290948 as early as possible after power on
 *
 */
void ICM_PowerOn(I2C_HandleTypeDef *hi2c) {
	ICM_SelectBank(hi2c, USER_BANK_0);
	HAL_Delay(5);
//	ICM_Enable_I2C(hi2c); // enable i2c master for mag
//	HAL_Delay(5);

//	ICM_SelectBank(hi2c, USER_BANK_3);
//	HAL_Delay(5);
//	ICM_Set_I2C_Clk(hi2c);
//	HAL_Delay(5);
//
//	ICM_SelectBank(hi2c, USER_BANK_0);
//	HAL_Delay(5);
	ICM_SetClock(hi2c, (uint8_t)CLK_BEST_AVAIL);
	HAL_Delay(5);
	ICM_AccelGyroOff(hi2c);
	HAL_Delay(20);
	ICM_AccelGyroOn(hi2c);
	HAL_Delay(35);
	ICM_Initialize(hi2c);
}

uint16_t ICM_Initialize(I2C_HandleTypeDef *hi2c) {
	ICM_SelectBank(hi2c, USER_BANK_2);
	HAL_Delay(5);
	ICM_SetGyroRateLPF(hi2c, GYRO_RATE_250, GYRO_LPF_17HZ);
	HAL_Delay(10);

	//Reset FIFO and set to stream mode
//	ICM_WriteOneByte(hi2c, 0x69, 0x0F);
//	HAL_Delay(5);
//	ICM_WriteOneByte(hi2c, 0x68, 0x0F);
//	HAL_Delay(5);
//
//	// Disable FIFO
//	ICM_WriteOneByte(hi2c, 0x66, 0x00);
//	HAL_Delay(5);
//	ICM_WriteOneByte(hi2c, 0x67, 0x0);
//	HAL_Delay(5);

	// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
	uint8_t i2cData = 0x0A;
	ICM_WriteOneByte(hi2c, 0x00, &i2cData);
	HAL_Delay(5);

	// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
	i2cData = (0x04 | 0x11);
	ICM_WriteOneByte(hi2c, 0x14, &i2cData);
	HAL_Delay(5);

	// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
	i2cData = 0x00;
	ICM_WriteOneByte(hi2c, 0x10, &i2cData);
	HAL_Delay(5);

	// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
	i2cData = 0x0A;
	ICM_WriteOneByte(hi2c, 0x11, &i2cData);
	HAL_Delay(5);

//	ICM_SelectBank(hi2c, USER_BANK_2);
//	HAL_Delay(20);

	// Configure AUX_I2C Magnetometer (onboard ICM-20948)
	i2cData = 0x00;
	ICM_WriteOneByte(hi2c, 0x7F, &i2cData); // Select user bank 0
	HAL_Delay(5);
	i2cData = 0x30;
	ICM_WriteOneByte(hi2c, 0x0F, &i2cData); // INT Pin / Bypass Enable Configuration
	i2cData = 0x20;
	ICM_WriteOneByte(hi2c, 0x03, &i2cData); // I2C_MST_EN
	i2cData = 0x30;
	ICM_WriteOneByte(hi2c, 0x7F, &i2cData); // Select user bank 3
	HAL_Delay(5);
	i2cData = 0x4D;
	ICM_WriteOneByte(hi2c, 0x01, &i2cData); // I2C Master mode and Speed 400 kHz
	ICM_Set_I2C_Clk(hi2c);
	i2cData = 0x01;
	ICM_WriteOneByte(hi2c, 0x02, &i2cData); // I2C_SLV0 _DLY_ enable
	i2cData = 0x81;
	ICM_WriteOneByte(hi2c, 0x05, &i2cData); // enable IIC	and EXT_SENS_DATA==1 Byte

	// Initialize magnetometer
	i2c_Mag_write(hi2c, 0x32, 0x01); // Reset AK8963
	HAL_Delay(1000);
	i2c_Mag_write(hi2c, 0x31, 0x02); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output

	return 1337;
}

void ICM_ReadAccelGyro(I2C_HandleTypeDef *hi2c) {
	uint8_t raw_data[12];
//	ICM_readBytes(hi2c, 0x2D, raw_data, 12);
	HAL_I2C_Mem_Read(hi2c, ICM20948_ADDRESS << 1, 0x2D, I2C_MEMADD_SIZE_8BIT, raw_data, 12, 1000); //read starting from ACCEL_XOUT_H

	accel_data[0] = (int16_t)(raw_data[0] << 8 | raw_data[1]);
	accel_data[1] = (int16_t)(raw_data[2] << 8 | raw_data[3]);
	accel_data[2] = (int16_t)(raw_data[4] << 8 | raw_data[5]);

	gyro_data[0] = (int16_t)(raw_data[6] << 8 | raw_data[7]);
	gyro_data[1] = (int16_t)(raw_data[8] << 8 | raw_data[9]);
	gyro_data[2] = (int16_t)(raw_data[10] << 8 | raw_data[11]);
}

void ICM_CorrectAccelGyro(I2C_HandleTypeDef *hi2c, int16_t raw_accel_data[3], int16_t raw_gyro_data[3]) {
//	corr_accel_data[0] = (raw_accel_data[0] - (accel_offset[0] / (1<<0x04))) * (1<<0x04) / 16384.0;
//	corr_accel_data[1] = (raw_accel_data[1] - (accel_offset[1] / (1<<0x04))) * (1<<0x04) / 16384.0;
//	corr_accel_data[2] = (raw_accel_data[2] - (accel_offset[2] / (1<<0x04))) * (1<<0x04) / 16384.0;

	corr_accel_data[0] = (raw_accel_data[0] - accel_offset[0]) / 4096.0f;
	corr_accel_data[1] = (raw_accel_data[1] - accel_offset[1]) / 4096.0f;
	corr_accel_data[2] = (raw_accel_data[2] - accel_offset[2]) / 4096.0f;

//	corr_gyro_data[0] = (raw_gyro_data[0] - (gyro_offset[0] / (1<<GYRO_RATE_250))) * (1<<GYRO_RATE_250) * 250.0 / 131000.0;
//	corr_gyro_data[1] = (raw_gyro_data[1] - (gyro_offset[1] / (1<<GYRO_RATE_250))) * (1<<GYRO_RATE_250) * 250.0 / 131000.0;
//	corr_gyro_data[2] = (raw_gyro_data[2] - (gyro_offset[2] / (1<<GYRO_RATE_250))) * (1<<GYRO_RATE_250) * 250.0 / 131000.0;


	corr_gyro_data[0] = (raw_gyro_data[0] - gyro_offset[0]) / 131.0f;
	corr_gyro_data[1] = (raw_gyro_data[1] - gyro_offset[1]) / 131.0f;
	corr_gyro_data[2] = (raw_gyro_data[2] - gyro_offset[2]) / 131.0f;

}

void ICM_SelectBank(I2C_HandleTypeDef *hi2c, uint8_t bank) {
	uint8_t i2cData = bank;
	ICM_WriteOneByte(hi2c, USER_BANK_SEL, &i2cData);
}

void ICM_Enable_I2C(I2C_HandleTypeDef *hi2c) { //user bank 0
	uint8_t i2cData = 0x20;
	ICM_WriteOneByte(hi2c, 0x03, &i2cData); // Enable I2C master
}

void ICM_SetClock(I2C_HandleTypeDef *hi2c, uint8_t clk) {
	uint8_t i2cData = clk;
	ICM_WriteOneByte(hi2c, PWR_MGMT_1, &i2cData);
}

void ICM_AccelGyroOff(I2C_HandleTypeDef *hi2c) { //user bank 0
	uint8_t i2cData = (0x38 | 0x07);
	ICM_WriteOneByte(hi2c, PWR_MGMT_2, &i2cData);
}

void ICM_AccelGyroOn(I2C_HandleTypeDef *hi2c) { //user bank 0
	uint8_t i2cData = 0x00;
	ICM_WriteOneByte(hi2c, PWR_MGMT_2, &i2cData);
}

uint8_t ICM_WHOAMI(I2C_HandleTypeDef *hi2c) {
	uint8_t i2cData = 0x01;
	ICM_ReadOneByte(hi2c, 0x00, &i2cData);
	return i2cData;
}

void ICM_SetGyroRateLPF(I2C_HandleTypeDef *hi2c, uint8_t rate, uint8_t lpf) {
	uint8_t i2cData = (rate|lpf);
	ICM_WriteOneByte(hi2c, GYRO_CONFIG_1, &i2cData);
}

void ICM_Set_I2C_Clk(I2C_HandleTypeDef *hi2c) { //user bank 3
	uint8_t i2cData = 0x07;
	ICM_WriteOneByte(hi2c, 0x01, &i2cData); //set I2C master clock to recommended freq
}

void ICM_Check_Mag_Overflow(I2C_HandleTypeDef *hi2c)
{
//	uint8_t i2cData = 0x08;
//	ICM_ReadOneByte(hi2c, 0x18, &i2cData);
//	return i2cData;
}

void ICM20948_Calibrate(I2C_HandleTypeDef *hi2c)
{
	ICM_SelectBank(hi2c, USER_BANK_0);
	HAL_Delay(10);

	// Calibrate accelerometer
	for(int i=0; i<50; i++){
		ICM_ReadAccelGyro(hi2c);
		accel_offset[0] += accel_data[0];
		accel_offset[1] += accel_data[1];
		accel_offset[2] += accel_data[2];

		gyro_offset[0] += gyro_data[0];
		gyro_offset[1] += gyro_data[1];
		gyro_offset[2] += gyro_data[2];

		HAL_Delay(12);
	}

	accel_offset[0] /= 50;
	accel_offset[1] /= 50;
	accel_offset[2] /= 50;
//	accel_offset[2] += 4096.0; // 4096 LSB/g

	// Calibrate gyroscope
//	for(int i=0; i<50; i++){
//		ICM_ReadAccelGyro(hi2c);
//
//		HAL_Delay(15);
//	}
//
	gyro_offset[0] /= 50;
	gyro_offset[1] /= 50;
	gyro_offset[2] /= 50;
}


void get_imu_data(I2C_HandleTypeDef *hi2c)
{
	ICM_ReadAccelGyro(hi2c);
	ICM_CorrectAccelGyro(hi2c, accel_data, gyro_data);
	dt = (double)(HAL_GetTick() - last_tick)/tick_rate;
	last_tick = HAL_GetTick();

	// Calculate Roll, Pitch, Yaw by integrating Gyro Data
	curr_pose.roll = corr_gyro_data[0]*dt/1000.0;
	curr_pose.pitch = corr_gyro_data[1]*dt/1000.0;
	curr_pose.yaw = corr_gyro_data[2]*dt/1000.0;
	curr_pose.dt = dt;
}

void reset_pose(){
	curr_pose.roll = 0;
	curr_pose.pitch = 0;
	curr_pose.yaw = 0;
}
void IMU_Init()
{
	tick_rate = HAL_GetTickFreq();
	last_tick = HAL_GetTick();
}
