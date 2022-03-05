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

// Define gyroscope and accelerometer registers
uint8_t ICM20948_ADDRESS = 0x68;
uint8_t REG_BANK_SEL = 0x7F;
uint8_t B0_GYRO_XOUT_H = 0x33;
uint8_t B0_ACCEL_XOUT_H = 0x2D;

// Define magnetometer registers
uint8_t AK09916_ADDR = 0x0C;
uint8_t MAG_HXL = 0x11;

