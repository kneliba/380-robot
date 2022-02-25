// Ultrasonic Sensor - HC-SR04
#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include "main.h"

// Information regarding to ultrasonic sensor
typedef struct
{
    GPIO_TypeDef*  TRIG_GPIO;
    uint16_t       TRIG_PIN;
    TIM_TypeDef*   TIM_Instance;
    uint32_t       IC_TIM_CH;
    uint32_t	   VAL1;
    uint32_t	   VAL2;
    uint32_t	   DIFFERENCE;
    uint8_t		   FIRST_CAPTURED;
    uint32_t	   DISTANCE;
}HCSR04_Type;

// Function Prototypes
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HCSR04_Read_Front (TIM_HandleTypeDef *htim);
void HCSR04_Read_Side (TIM_HandleTypeDef *htim);

#endif /* ULTRASONIC_H */
