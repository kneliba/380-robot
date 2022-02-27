// Read gyroscope
void ICM20948_Read_Gyro();

// Read accelerometer
void ICM20948_Read_Accel();

// Read magnetometer
void ICM20948_Read_Magn();

// Axis typedef
typedef struct
{
	float x;
	float y;
	float z;
} axises;

// Define read write addresses
#define READ							0x80
#define WRITE							0x00

// Define magnetometer registers
#define AK09916_ADDRESS 				0x0C
#define MAG_HXL							0x11

