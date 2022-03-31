/*
 * ICM20948.h
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

#ifndef IMU_H
#define IMU_H
#include "main.h"

extern int16_t accel_data[3];
extern int16_t gyro_data[3];
extern int16_t mag_data[3];

extern float corr_accel_data[3];
extern float corr_gyro_data[3];

typedef struct
{
    float roll;
    float pitch;
    float yaw;

    uint16_t dt;

}robot_pose;

extern robot_pose curr_pose;

#define UART_BUS		    (&huart2)

#define ICM20948_ADDRESS    (0x69)
#define AK09916_ADDRESS		(0X0C)

#define USER_BANK_SEL   	(0x7F)
#define USER_BANK_0		    (0x00)
#define USER_BANK_1		    (0x10)
#define USER_BANK_2		    (0x20)
#define USER_BANK_3		    (0x30)

#define PWR_MGMT_1 		    (0x06)
#define PWR_MGMT_2		    (0x07)
#define GYRO_CONFIG_1	    (0x01)


#define CLK_BEST_AVAIL   	(0x01)
#define GYRO_RATE_250	    (0x00)
#define GYRO_LPF_17HZ   	(0x29)

void ICM_WriteOneByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *pData);
void ICM_readBytes(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *pData, uint16_t Size);
void ICM_ReadOneByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t* pData);
void ICM_PowerOn(I2C_HandleTypeDef *hi2c);
uint8_t ICM_WHOAMI(I2C_HandleTypeDef *hi2c);
void ICM_SelectBank(I2C_HandleTypeDef *hi2c, uint8_t bank);
void ICM_ReadAccelGyro(I2C_HandleTypeDef *hi2c);
void ICM_ReadMag(I2C_HandleTypeDef *hi2c, int16_t magn[3]);
uint16_t ICM_Initialize(I2C_HandleTypeDef *hi2c);
void ICM_SelectBank(I2C_HandleTypeDef *hi2c, uint8_t bank);
void ICM_Enable_I2C(I2C_HandleTypeDef *hi2c);
void ICM_SetClock(I2C_HandleTypeDef *hi2c, uint8_t clk);
void ICM_AccelGyroOff(I2C_HandleTypeDef *hi2c);
void ICM_AccelGyroOn(I2C_HandleTypeDef *hi2c);
void ICM_SetGyroRateLPF(I2C_HandleTypeDef *hi2c, uint8_t rate, uint8_t lpf);
void ICM_SetGyroLPF(uint8_t lpf);
void ICM_Set_I2C_Clk(I2C_HandleTypeDef *hi2c);
void ICM20948_Calibrate(I2C_HandleTypeDef *hi2c);
void ICM_CorrectAccelGyro(I2C_HandleTypeDef *hi2c, int16_t raw_accel_data[3], int16_t raw_gyro_data[3]);
void get_imu_data(I2C_HandleTypeDef *hi2c);

#endif /* IMU_H */
