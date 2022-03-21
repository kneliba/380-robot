/*  AUXILIARY FUNCTIONS
 *  - Read Battery Voltage
 *  - Button Input
 */

#ifndef AUX_FUNCTIONS_H
#define AUX_FUNCTIONS_H
#include "main.h"

#define voltage_divder_factor 0.25
#define voltage_offset_volts -0.2

#define VREF 3.25

float read_batt_voltage(ADC_HandleTypeDef* hadc);

#endif
