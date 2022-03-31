/*
 * ESP.c
 *
 *  Created on: Mar 30, 2022
 *      Author: Dawson Kletke
 */
#include "ESP.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f103xb.h"
#include <string.h>
#include <stdlib.h>


uint8_t UART2_rxBuffer[RX_BUFF_SIZE] = {0};
uint8_t MSG[35] = {'\0'};

char drive_forward_com[] = "df";
char drive_distance_com[] = "dd";
char drive_until_com[] = "du";

char stop_com[] = "stop";

char turn_right_com[] = "tr";
char turn_degree_com[] = "td";

char accelerate_com[] = "ac";
char decelerate_com[] = "decel";
char adapt_decel_com[] = "a_dec";

char P_set_com[] = "P_kp";
char PID_set_kp_com[] = "PID_kp";
char PID_set_ki_com[] = "PID_ki";
char PID_set_kd_com[] = "PID_kd";


void ESP_Receive(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, uint8_t *UART2_rxBuffer, UART_HandleTypeDef *huart2) {
	//esp command: "df_030" where 030 is the speed percentage
	//	char *received_buff = (char*)UART2_rxBuffer;
	if(strncmp((char *)UART2_rxBuffer, drive_forward_com, strlen(drive_forward_com)) == 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, drive_forward_com);
		drive_forward(htim2, speed);

		sprintf(MSG, "Command received: %s with %d \n", drive_forward_com, speed);
		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "dd_030_2.15" where 030 is the speed percentage, 2.15 is distance in m
	else if(strncmp((char *)UART2_rxBuffer, drive_distance_com, strlen(drive_distance_com)) == 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, drive_distance_com);
		double distance = get_double_from_string((char *)UART2_rxBuffer, drive_distance_com, 5);

		drive_distance(htim1, htim2, hi2c2, speed, distance);

		sprintf(MSG, "Command received: %s with %d \n", drive_distance_com, speed);
		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "du_030_2.15" where 030 is the speed percentage, 2.15 is distance in m
	else if(strncmp((char *)UART2_rxBuffer, drive_until_com, strlen(drive_until_com)) == 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, drive_until_com);
		double distance = get_double_from_string((char *)UART2_rxBuffer, drive_until_com, 5);

		drive_until(htim2, htim3, hi2c2, speed, distance);

		sprintf(MSG, "Command received: %s with %d \n", drive_until_com, speed);
		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "stop--" add dashes so length is met
	else if(strncmp((char *)UART2_rxBuffer, stop_com, strlen(stop_com))== 0) {
		stop(htim2);

		sprintf(MSG, "Command received: %s\n", stop_com);
		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "tr_030"
	else if(strncmp((char *)UART2_rxBuffer, turn_right_com, strlen(turn_right_com))== 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, turn_right_com);
		turn_right(htim2, speed);

		sprintf(MSG, "Command received: %s\n", turn_right_com);
		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "td_090" where 090 is the angle
	else if(strncmp((char *)UART2_rxBuffer, turn_right_com, strlen(turn_degree_com))== 0) {
		int angle = get_integer_from_string((char *)UART2_rxBuffer, turn_degree_com);
		turn_degree(htim2, hi2c2, angle);

		sprintf(MSG, "Command received: %s\n", turn_degree_com);
		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "ac_030" where 030 is the speed percentage
	else if(strncmp((char *)UART2_rxBuffer, accelerate_com, strlen(accelerate_com))== 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, accelerate_com);
		accelerate(htim2, speed);

		sprintf(MSG, "Command received: %s with %d \n", accelerate_com, speed);
		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "decel" add dashes so length is met
	else if(strncmp((char *)UART2_rxBuffer, decelerate_com, strlen(decelerate_com))== 0) {
		decelerate(htim2);

		sprintf(MSG, "Command received: %s\n", decelerate_com);
		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "adec_030_2.15" where 030 is the speed percentage, 2.15 is distance in m
	else if(strncmp((char *)UART2_rxBuffer, adapt_decel_com, strlen(adapt_decel_com))== 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, adapt_decel_com);
		double distance = get_double_from_string((char *)UART2_rxBuffer, drive_distance_com, 5);

		adapt_decel(htim2, htim3, hi2c2, speed, distance);

		sprintf(MSG, "Command received: %s with %d \n", adapt_decel_com, speed);
		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "P_kp_8.00"
	else if(strncmp((char *)UART2_rxBuffer, P_set_com, strlen(P_set_com))== 0) {
		double kp = get_double_from_string((char *)UART2_rxBuffer, P_set_com, 1);
		set_P_control_Kp(kp);
	}

	//esp command: "PID_kp_8.00'
	else if(strncmp((char *)UART2_rxBuffer, PID_set_kp_com, strlen(PID_set_kp_com))== 0) {
		double kp = get_double_from_string((char *)UART2_rxBuffer, PID_set_kp_com, 1);
		set_PID_Kp(kp);
	}

	//esp command: "PID_ki_8.00'
	else if(strncmp((char *)UART2_rxBuffer, PID_set_ki_com, strlen(PID_set_ki_com))== 0) {
		double ki = get_double_from_string((char *)UART2_rxBuffer, PID_set_ki_com, 1);
		set_PID_Ki(ki);
	}

	//esp command: "PID_ki_8.00'
	else if(strncmp((char *)UART2_rxBuffer, PID_set_kd_com, strlen(PID_set_kd_com))== 0) {
		double kd = get_double_from_string((char *)UART2_rxBuffer, PID_set_kd_com, 1);
		set_PID_Kd(kd);
	}

}

//3 digits numbers currently
int get_integer_from_string(char *buffer_msg, char *string_command){

	char int_substr[4];
	memcpy(int_substr, &buffer_msg[strlen(string_command)+1], 3 );
	int_substr[3] = '\0';

	int int_value = atoi(int_substr);
	return int_value;
}

//3 digits numbers with decimal currently: "2.15"
double get_double_from_string(char *buffer_msg, char *string_command, int offset){
	char double_substr[5];
	memcpy(double_substr, &buffer_msg[strlen(string_command)+offset], 4 );
	double_substr[4] = '\0';

	double double_value = atof(double_substr);
	return double_value;
}





