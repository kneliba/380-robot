/*
 * ESP.h
 *
 *  Created on: Mar 30, 2022
 *      Author: Dawson Kletke
 */

#ifndef INC_ESP_H_
#define INC_ESP_H_

#include "main.h"

#define RX_BUFF_SIZE 8
#define TX_BUFF_SIZE 64

extern uint8_t UART2_rxBuffer[8];


void ESP_Receive(TIM_HandleTypeDef *htim, uint8_t *UART2_rxBuffer, UART_HandleTypeDef *huart2);
int get_integer_from_string(char *buffer_msg, char *string_command);

#endif /* INC_ESP_H_ */
