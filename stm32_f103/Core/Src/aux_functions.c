/*
 * aux_functions.c
 *
 *  Created on: Mar 21, 2022
 *      Author: Joseph
 */

#include <aux_functions.h>
#include <main.h>


// Read battery
float read_batt_voltage(ADC_HandleTypeDef* hadc){
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	uint32_t raw_val = HAL_ADC_GetValue(hadc);

	float voltage_raw = (raw_val/4095.0/voltage_divder_factor)*VREF;

	float batt_voltage = voltage_raw + voltage_offset_volts;

	return batt_voltage;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == SWITCH_Pin){
		// Custom function here
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}
