//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern float beta;				// algorithm gain
extern float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern float roll, pitch, yaw;
extern float w_bx, w_by, w_bz;


//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void computeAngles();
float getRoll();
float getPitch();
float getYaw();

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
