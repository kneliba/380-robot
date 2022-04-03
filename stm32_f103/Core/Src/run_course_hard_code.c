#include "main.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f103xb.h"
#include "ESC.h"
#include "ultrasonic.h"

// TODO: test how much distance is required to turn
static double dist_to_turn = 11;
static double dist_of_tile = 30.5;
static double dist_beside_wall = 15;
static uint16_t fast = 15;
static uint16_t slow = 12;
static uint16_t faster = 22;

// number of tiles to wall
double distances_to_wall[11] = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2};
// number of tiles to travel
double distances_to_travel[11] = {4, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1};
// distance to side wall
double distances_to_side[11] = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2};

void drive_course(TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, double speed) {

	//segment 1
//	reset_pose();
//	drive_until(htim3, hi2c2, fast, dist_to_turn, dist_beside_wall);
//	turn_degree(hi2c2, 90);
//
//	//segment 2
//	reset_pose();
//	drive_until(htim3, hi2c2, fast, dist_to_turn, dist_beside_wall);
//	turn_degree(hi2c2, 90);
//
//	//segment 3
//	reset_pose();
//	drive_until(htim3, hi2c2, fast, dist_to_turn, dist_beside_wall);
//	turn_degree(hi2c2, 90);
//
//	//segment 4
//	reset_pose();
//	drive_pit(htim3, hi2c2, fast, dist_beside_wall);
//	drive_until(htim3, hi2c2, fast, dist_to_turn+dist_of_tile, dist_beside_wall);
//	turn_degree(hi2c2, 90);

	//segment 5 (2 single gravel)
	reset_pose();
	drive_until(htim3, hi2c2, faster, dist_to_turn+dist_of_tile+5, dist_beside_wall+dist_of_tile);
	turn_degree(hi2c2, 90);

	//segment 6 (long gravel)
	reset_pose();
	drive_until(htim3, hi2c2, faster, dist_to_turn+dist_of_tile+5, dist_beside_wall+dist_of_tile);
	turn_degree(hi2c2, 90);

	//segment 7 (long sand)
	reset_pose();
//	drive_pit(htim3, hi2c2, fast, dist_beside_wall+dist_of_tile);
	drive_until(htim3, hi2c2, faster, dist_to_turn+dist_of_tile+5, dist_beside_wall+dist_of_tile);
	turn_degree(hi2c2, 90);

	//segment 8 (single sand)
	reset_pose();
//	drive_pit(htim3, hi2c2, fast, dist_beside_wall+dist_of_tile);
	drive_until(htim3, hi2c2, faster, dist_to_turn+dist_of_tile+dist_of_tile+5, dist_beside_wall+dist_of_tile);
	turn_degree(hi2c2, 90);

	//segment 9 (single sand)
	reset_pose();
//	drive_pit(htim3, hi2c2, fast, dist_beside_wall+dist_of_tile);
	drive_until(htim3, hi2c2, faster, dist_to_turn+dist_of_tile+dist_of_tile+5, dist_beside_wall+dist_of_tile+dist_of_tile);
	turn_degree(hi2c2, 90);

	//segment 10
	reset_pose();
	drive_until(htim3, hi2c2, slow, dist_to_turn+dist_of_tile+dist_of_tile, dist_beside_wall+dist_of_tile+dist_of_tile);
	turn_degree(hi2c2, 90);

	//segment 11
	reset_pose();
	drive_until(htim3, hi2c2, slow, dist_to_turn+dist_of_tile+dist_of_tile, dist_beside_wall+dist_of_tile+dist_of_tile);
}


