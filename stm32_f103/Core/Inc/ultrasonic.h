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
    double	   	   VAL1;
    double	   	   VAL2;
    double	       DIFFERENCE;
    uint8_t		   FIRST_CAPTURED;
    double	   	   DISTANCE;
}HCSR04_Type;

// Variables for filtering
static const double r = 0.01;

// Function Prototypes
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HCSR04_Read_Front (TIM_HandleTypeDef *htim);
void HCSR04_Read_Side (TIM_HandleTypeDef *htim);
double get_front_distance (void);
double get_side_distance (void);
double filter (double sensor_val, double old_Kv);

#endif /* ULTRASONIC_H */
