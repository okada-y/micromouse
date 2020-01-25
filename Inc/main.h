/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Sensor_FL_Pin GPIO_PIN_0
#define Sensor_FL_GPIO_Port GPIOA
#define Sensor_SL_Pin GPIO_PIN_1
#define Sensor_SL_GPIO_Port GPIOA
#define Sensor_SR_Pin GPIO_PIN_2
#define Sensor_SR_GPIO_Port GPIOA
#define Sensor_FR_Pin GPIO_PIN_3
#define Sensor_FR_GPIO_Port GPIOA
#define LED_FL_Pin GPIO_PIN_4
#define LED_FL_GPIO_Port GPIOA
#define LED_SL_Pin GPIO_PIN_5
#define LED_SL_GPIO_Port GPIOA
#define LED_SR_Pin GPIO_PIN_6
#define LED_SR_GPIO_Port GPIOA
#define LED_FR_Pin GPIO_PIN_7
#define LED_FR_GPIO_Port GPIOA
#define V_Battery_Pin GPIO_PIN_0
#define V_Battery_GPIO_Port GPIOB
#define Push_SW_Pin GPIO_PIN_1
#define Push_SW_GPIO_Port GPIOB
#define gyro_CS_Pin GPIO_PIN_12
#define gyro_CS_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_8
#define LED5_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOA
#define L_Forword_Pin GPIO_PIN_6
#define L_Forword_GPIO_Port GPIOB
#define L_Reverse_Pin GPIO_PIN_7
#define L_Reverse_GPIO_Port GPIOB
#define R_Forword_Pin GPIO_PIN_8
#define R_Forword_GPIO_Port GPIOB
#define R_Reverse_Pin GPIO_PIN_9
#define R_Reverse_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
