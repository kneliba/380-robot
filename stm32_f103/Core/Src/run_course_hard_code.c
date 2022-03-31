#include "main.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
#include "ESC.h"
#include "ultrasonic.h"

// TODO: test how much distance is required to turn
static double dist_to_turn = 15;
static double dist_of_tile = 30.5;
static double dist_beside_wall = 10;

// number of tiles to wall
double distances_to_wall[11] = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2};
// number of tiles to travel
double distances_to_travel[11] = {4, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1};
// distance to side wall
double distances_to_side[11] = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2};

void drive_course(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed) {

	//segment 1
	drive_until(htim2,htim3, hi2c2, speed, dist_to_turn);
	turn_degree(htim2, hi2c2, 90);

	//segment 2
	drive_until(htim2,htim3, hi2c2, speed, dist_to_turn);
	turn_degree(htim2, hi2c2, 90);

	//segment 3
	drive_until(htim2,htim3, hi2c2, speed, dist_to_turn);
	turn_degree(htim2, hi2c2, 90);

	//segment 4
	drive_pit(htim2, htim3, hi2c2, speed);
	drive_until(htim2, htim3, hi2c2, speed, dist_to_turn+dist_of_tile);
	turn_degree(htim2, hi2c2, 90);

	//segment 5 (2 single gravel)
	// driving slower for a shorter distance
	drive_until(htim2,htim3, hi2c2, speed*0.8, dist_to_turn+dist_of_tile);
	turn_degree(htim2, hi2c2, 90);

	//segment 6 (long gravel)
	drive_until(htim2,htim3, hi2c2, speed*0.8, dist_to_turn+dist_of_tile);
	turn_degree(htim2, hi2c2, 90);

	//segment 7 (long sand)
	drive_pit(htim2, htim3, hi2c2, speed);
	drive_until(htim2, htim3, hi2c2, speed*0.8, dist_to_turn+dist_of_tile);
	turn_degree(htim2, hi2c2, 90);

	//segment 8 (single sand)
	drive_pit(htim2, htim3, hi2c2, speed);
	drive_until(htim2, htim3, hi2c2, speed*0.8, dist_to_turn+dist_of_tile+dist_of_tile);
	turn_degree(htim2, hi2c2, 90);

	//segment 9 (single sand)
	drive_pit(htim2, htim3, hi2c2, speed);
	drive_until(htim2, htim3, hi2c2, speed*0.8, dist_to_turn+dist_of_tile+dist_of_tile);
	turn_degree(htim2, hi2c2, 90);

	//segment 10
	drive_until(htim2,htim3, hi2c2, speed, dist_to_turn+dist_of_tile+dist_of_tile);
	turn_degree(htim2, hi2c2, 90);

	//segment 11
	drive_until(htim2,htim3, hi2c2, speed, dist_to_turn+dist_of_tile+dist_of_tile);
}

void drive_pit (TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed) {
	bool pitched_down = false;
	bool pitched_up = false;
	double curr_pitch = 0;

	while(!pitched_down){
		get_imu_data(hi2c2);
		curr_pitch += curr_pose.pitch;

		drive_until(htim2, htim3, hi2c2, speed, dist_to_turn);
		if(curr_pitch < 0){
			pitched_down = true;
		}
	}
	//speed up inside pit
	while(!pitched_up){
		get_imu_data(hi2c2);
		curr_pitch += curr_pose.pitch;

		accelerate(htim2, speed*1.2);
		if(curr_pitch > 0){
			pitched_up = true;
		}
	}
	//still on an angle, finish climbing
	while(curr_pitch > 1){
		get_imu_data(hi2c2);
		curr_pitch += curr_pose.pitch;

		drive_until(htim2,htim3, hi2c2, speed*1.2, dist_to_turn);
	}
}
