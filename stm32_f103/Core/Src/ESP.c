/*
 * ESP.c
 *
 *  Created on: Mar 22, 2022
 *      Author: junepark
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
char drive_forward_com[] = "df";
char stop_com[] = "stop";
char turn_right_com[] = "turn_r";
char accelerate_com[] = "acc";
char decelerate_com[] = "decel";


void ESP_Receive(TIM_HandleTypeDef *htim, uint8_t *UART2_rxBuffer) {
	//esp command: "df_030 " where 030 is the speed percentage
//	char *received_buff = (char*)UART2_rxBuffer;
	if(strncmp((char *)UART2_rxBuffer, drive_forward_com, strlen(drive_forward_com)) == 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, drive_forward_com);

		drive_forward(htim, speed);
	}

	//esp command: "stop--" add dashes so length is met
	else if(strncmp((char *)UART2_rxBuffer, stop_com, strlen(stop_com))== 0) {
		stop(htim);
	}

	//esp command: "turn_r"
	else if(strncmp((char *)UART2_rxBuffer, turn_right_com, strlen(turn_right_com))== 0) {
		turn_right(htim);
	}

	//esp command: "acc_030 " where 030 is the speed percentage
	else if(strncmp((char *)UART2_rxBuffer, accelerate_com, strlen(accelerate_com))== 0) {
		int speed = get_integer_from_string((char *)UART2_rxBuffer, accelerate_com);

		accelerate(htim, speed);
	}

	//esp command: "decel-" add dashes so length is met
	else if(strncmp((char *)UART2_rxBuffer, decelerate_com, strlen(decelerate_com))== 0) {
		decelerate(htim);
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

