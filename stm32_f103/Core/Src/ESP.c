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

uint8_t UART2_rxBuffer[35] = {0};

void ESP_Receive(TIM_HandleTypeDef *htim) {
	if(strcmp(UART2_rxBuffer, "drive forward") == 0) {
		drive_forward(htim, speed);
	}
	else if(strcmp(UART2_rxBuffer, "stop")== 0) {
		stop(htim);
	}
}

