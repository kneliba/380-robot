// Axis typedef
typedef struct
{
	float x;
	float y;
	float z;
} axises;

// Read gyroscope
axises ICM20948_Read_Gyro();

// Read accelerometer
axises ICM20948_Read_Accel();

// Read magnetometer
axises ICM20948_Read_Magn();

// Calibrate gyroscope and accelerometer
void ICM20948_Calibrate();

// Define read write addresses
#define READ							0x80
#define WRITE							0x00

#define B0_GYRO_XOUT_H					0x33
#define B0_ACCEL_XOUT_H					0x2D

// Define magnetometer registers
#define AK09916_ADDR 				0x0C
#define MAG_HXL							0x11

