/*
 * run_course.c
 *
 *  Created on: Mar. 25, 2022
 *      Author: junepark
 */

#include "run_course.h"
#include "main.h"
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

// keep track of where robot is on course
int8_t course[6][6] = {{0, 0, 0, 0, 0, 0},
					   {0, 0, 0, 0, 0, 0},
					   {0, 0, 0, 0, 0, 0},
					   {0, 0, 0, 0, 0, 0},
					   {0, 0, 0, 0, 0, 0},
					   {0, 0, 0, 0, 1, 0}
					   };
int8_t current_pos[2] = {5,4};
int8_t tiles_travelled_total = 0;
int8_t full_path[35][2] = {{5,4}, {5,3}, {5,2}, {5,1}, {5,0},
						   {4,0}, {3,0}, {2,0}, {1,0}, {0,0},
						   {0,1}, {0,2}, {0,3}, {0,4}, {0,5},
						   {1,5}, {2,5}, {3,5}, {4,5},
						   {4,4}, {4,3}, {4,2}, {4,1},
						   {3,1}, {2,1}, {1,1},
						   {1,2}, {1,3}, {1,4},
						   {2,4}, {3,4},
						   {3,3}, {3,2},
						   {2,2},
						   {2,3}
						  };

int8_t pits[3][2] = {{2, 5},
					 {3, 5},
					 {4, 4}
					};
int8_t gravel[3][2] = {{2, 1},
					   {3, 1},
					   {4, 2}
					  };
int8_t sand[4][2] = {{1, 2},
					 {1, 3},
					 {2, 4},
					 {3, 3}
					};

// run full course by distance using ultrasonic only
void run_course_ultrasonic (TIM_HandleTypeDef *htim, double drive_speed, double turn_speed)
{
	// calculate distances to wall in cm
	for(int8_t i = 0; i < 11; i++)
	{
		distances_to_wall[i] *= dist_of_tile;
		distances_to_wall[i] += dist_to_turn;
	}

	int8_t current = 0;

	while(current < 11)
	{
		// error correction, use angle or side ultrasonic to fix angle
		drive_until(drive_speed, distances_to_wall[current]);
		turn_right(htim, turn_speed);
		current++;
	}
}

// run full course by distance using motor encoders only
void run_course_encoders (TIM_HandleTypeDef *htim, double drive_speed, double turn_speed)
{
	// calculate distance to travel in cm
	for(int8_t i = 0; i < 11; i++)
	{
		distances_to_travel[i] *= dist_of_tile;
	}

	int8_t current = 0;

	while(current < 11)
	{
		// error correction, use angle or side ultrasonic to fix angle
		drive_distance(drive_speed, distances_to_travel[current]);
		turn_right(htim, turn_speed);
		current++;
	}
}

// run full course by distance using motor encoders and ultrasonic
void run_course_encoders_ultrasonic (TIM_HandleTypeDef *htim, double drive_speed, double turn_speed)
{
	int8_t tiles_travelled_segment = 0;

	// calculate distance to travel in cm
	for(int8_t i = 0; i < 11; i++)
	{
		distances_to_travel[i] *= dist_of_tile;
		distances_to_side[i] *= dist_of_tile;
		distances_to_side[i] += dist_beside_wall;
	}

	int8_t current = 0;

	while(current < 11)
	{
		reset_distance(htim);
		int16_t  encoder_dist = get_distance_travelled();
		int16_t  ultrasonic_dist = get_front_distance();
		int16_t  dist_of_tile_travelled = get_distance_travelled();

		while (encoder_dist < distances_to_travel[current] && ultrasonic_dist > dist_to_turn) {
			if(dist_of_tile_travelled > dist_of_tile) {
				current_pos[0] = full_path[tiles_travelled_total][0];
				current_pos[1] = full_path[tiles_travelled_total][1];
				tiles_travelled_segment++;
				tiles_travelled_total++;
				dist_of_tile_travelled = encoder_dist - dist_of_tile*tiles_travelled_segment;
			}

			if(current_pos == pits[0] || current_pos == pits[1] || current_pos == pits[3]) { // also in sand || gravel
				drive_distance(drive_speed, distances_to_travel[current]);
			}
			else {
				drive_straight_ultrasonic(htim, drive_speed, dist_of_tile);
			}
			encoder_dist = get_distance_travelled();
			ultrasonic_dist = get_front_distance();
		}

		tiles_travelled_segment = 0;

		stop(htim);
		turn_right(htim, turn_speed);
		current++;
	}
}


