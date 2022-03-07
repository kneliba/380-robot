#ifndef IMU_H
#define IMU_H
#include "main.h"

// Typedefs
typedef struct
{
	float x;
	float y;
	float z;
} axises;

typedef enum
{
	ub_0 = 0 << 4,
	ub_1 = 1 << 4,
	ub_2 = 2 << 4,
	ub_3 = 3 << 4
} userbank;

// Read gyroscope
axises ICM20948_Read_Gyro(I2C_HandleTypeDef *hi2c1);

// Read accelerometer
axises ICM20948_Read_Accel(I2C_HandleTypeDef *hi2c1);

// Read magnetometer
axises ICM20948_Read_Magn(I2C_HandleTypeDef *hi2c1);

// Calibrate gyroscope and accelerometer
void ICM20948_Calibrate(I2C_HandleTypeDef *hi2c1);

#endif /* IMU_H */
