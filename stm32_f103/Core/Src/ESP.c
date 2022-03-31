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
char adapt_decel_com[] = "a_decel";

char Pcontr_set_kp[] = "Pcontr_set_kp";
char PIDcon_get_kp[] = "PIDcon_set_kp";

void ESP_Receive(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, I2C_HandleTypeDef *hi2c2, uint8_t *UART2_rxBuffer, UART_HandleTypeDef *huart2) {
	//esp command: "df_030" where 030 is the speed percentage
	//	char *received_buff = (char*)UART2_rxBuffer;
	if(strncmp((char *)UART2_rxBuffer, drive_forward_com, strlen(drive_forward_com)) == 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, drive_forward_com);
		drive_forward(htim2, speed);
//		sprintf(MSG, "Command received: %s with %d \n", drive_forward_com, speed);
//		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "dd_030_2.15" add dashes so length is met
	else if(strncmp((char *)UART2_rxBuffer, drive_distance_com, strlen(drive_distance_com)) == 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, drive_distance_com);
		double distance = get_double_from_string((char *)UART2_rxBuffer, drive_distance_com);
		drive_distance(htim1, htim2, hi2c2, speed, distance);
//		sprintf(MSG, "Command received: %s with %d \n", drive_distance_com, speed);
//		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "stop--" add dashes so length is met
	else if(strncmp((char *)UART2_rxBuffer, stop_com, strlen(stop_com))== 0) {
		stop(htim2);
//		sprintf(MSG, "Command received: %s\n", stop_com);
//		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "tr_030"
	else if(strncmp((char *)UART2_rxBuffer, turn_right_com, strlen(turn_right_com))== 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, turn_right_com);
		turn_right(htim2, speed);
//		sprintf(MSG, "Command received: %s\n", turn_right_com);
//		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "ac_030" where 030 is the speed percentage
	else if(strncmp((char *)UART2_rxBuffer, accelerate_com, strlen(accelerate_com))== 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, accelerate_com);
		accelerate(htim2, speed);

//		sprintf(MSG, "Command received: %s with %d \n", accelerate_com, speed);
//		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
	}

	//esp command: "decel-" add dashes so length is met
	else if(strncmp((char *)UART2_rxBuffer, decelerate_com, strlen(decelerate_com))== 0) {
		decelerate(htim2);

//		sprintf(MSG, "Command received: %s\n", decelerate_com);
//		HAL_UART_Transmit(huart2, MSG, sizeof(MSG), 100);
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
double get_double_from_string(char *buffer_msg, char *string_command){
	char double_substr[5];
	memcpy(double_substr, &buffer_msg[strlen(string_command)+5], 4 );
	double_substr[4] = '\0';

	double double_value = atof(double_substr);
	return double_value;
}


