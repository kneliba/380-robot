/*
 * ESP.h
 *
 *  Created on: Mar 22, 2022
 *      Author: junepark
 */

#ifndef INC_ESP_H_
#define INC_ESP_H_
#include "main.h"

extern uint8_t UART2_rxBuffer[35];

void ESP_Receive(TIM_HandleTypeDef *htim);

#endif /* INC_ESP_H_ */
