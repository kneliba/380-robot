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

extern HCSR04_Type Front_US;
extern HCSR04_Type Side_US;

// Variables for filtering
static const double r = 0.01;

// distances for ideal robot distance from side of the walls
static const double ideal_1st_block_distance = 10;
static const  double ideal_2nd_block_distance = 40.5;
static const double ideal_3rd_block_distance = 70.5;


// Function Prototypes
void HCSR04_Read_Front (TIM_HandleTypeDef *htim);
void HCSR04_Read_Side (TIM_HandleTypeDef *htim);
double get_front_distance (void);
double get_side_distance (void);
double filter (double sensor_val, double old_Kv);

#endif /* ULTRASONIC_H */
