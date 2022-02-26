/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEFT_PWM_Pin GPIO_PIN_0
#define LEFT_PWM_GPIO_Port GPIOA
#define RIGHT_PWM_Pin GPIO_PIN_1
#define RIGHT_PWM_GPIO_Port GPIOA
#define BATTERY_Pin GPIO_PIN_4
#define BATTERY_GPIO_Port GPIOA
#define SWITCH_Pin GPIO_PIN_5
#define SWITCH_GPIO_Port GPIOA
#define FRONT_TRIG_Pin GPIO_PIN_6
#define FRONT_TRIG_GPIO_Port GPIOA
#define FRONT_ECHO_Pin GPIO_PIN_7
#define FRONT_ECHO_GPIO_Port GPIOA
#define SIDE_ECHO_Pin GPIO_PIN_0
#define SIDE_ECHO_GPIO_Port GPIOB
#define SIDE_TRIG_Pin GPIO_PIN_1
#define SIDE_TRIG_GPIO_Port GPIOB
#define I2C_IMU_SCL_Pin GPIO_PIN_10
#define I2C_IMU_SCL_GPIO_Port GPIOB
#define I2C_IMU_SDA_Pin GPIO_PIN_11
#define I2C_IMU_SDA_GPIO_Port GPIOB
#define RIGHT_ENCODER_A_Pin GPIO_PIN_9
#define RIGHT_ENCODER_A_GPIO_Port GPIOA
#define RIGHT_ENCODER_B_Pin GPIO_PIN_10
#define RIGHT_ENCODER_B_GPIO_Port GPIOA
#define LEFT_ENCODER_A_Pin GPIO_PIN_11
#define LEFT_ENCODER_A_GPIO_Port GPIOA
#define LEFT_ENCODER_B_Pin GPIO_PIN_12
#define LEFT_ENCODER_B_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_3
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_6
#define LED_B_GPIO_Port GPIOB
#define I2C_ESP_SCL_Pin GPIO_PIN_8
#define I2C_ESP_SCL_GPIO_Port GPIOB
#define I2C_ESP_SDA_Pin GPIO_PIN_9
#define I2C_ESP_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
