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
    float	   	   DISTANCE;
    float		   SENSOR_VAL;
}HCSR04_Type;

extern HCSR04_Type Front_US;
extern HCSR04_Type Side_US;

// Variables for filtering
static const double r = 0.01;

// Function Prototypes
void HCSR04_Read_Front (TIM_HandleTypeDef *htim);
void HCSR04_Read_Side (TIM_HandleTypeDef *htim);
float get_front_distance (void);
float get_side_distance (void);
float filter (double sensor_val, double old_Kv);

#endif /* ULTRASONIC_H */
