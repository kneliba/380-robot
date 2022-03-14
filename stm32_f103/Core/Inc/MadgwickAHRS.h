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
#include "main.h"

//----------------------------------------------------------------------------------------------------
// Variable declaration

uint32_t beta;				// algorithm gain
uint32_t q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
uint32_t roll;
uint32_t pitch;
uint32_t yaw;
char anglesComputed;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(uint32_t gx, uint32_t gy, uint32_t gz, uint32_t ax, uint32_t ay, uint32_t az, uint32_t mx, uint32_t my, uint32_t mz);
void MadgwickAHRSupdateIMU(uint32_t gx, uint32_t gy, uint32_t gz, uint32_t ax, uint32_t ay, uint32_t az);
void computeAngles();
uint32_t getRoll();
uint32_t getPitch();
uint32_t getYaw();

#endif
