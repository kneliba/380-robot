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
    double	   	   DISTANCE;
    double		   SENSOR_VAL;
}HCSR04_Type;

extern HCSR04_Type Front_US;
extern HCSR04_Type Side_US;

// Variables for filtering
static const double r = 0.5;

// Function Prototypes
void HCSR04_timer_input_CC (TIM_HandleTypeDef *htim);
void HCSR04_Read_Front (TIM_HandleTypeDef *htim);
void HCSR04_Read_Side (TIM_HandleTypeDef *htim);
double get_front_distance (void);
double get_side_distance (void);
double filter (double sensor_val, double old_Kv);

#endif /* ULTRASONIC_H */
