/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g0xx_hal.h"

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
#define FlowSensor_Tach_Pin GPIO_PIN_5
#define FlowSensor_Tach_GPIO_Port GPIOA
#define FlowSensor_Tach_EXTI_IRQn EXTI4_15_IRQn
#define Fan1_Tach_Pin GPIO_PIN_6
#define Fan1_Tach_GPIO_Port GPIOA
#define Fan1_Tach_EXTI_IRQn EXTI4_15_IRQn
#define Fan1_PWM__Pin GPIO_PIN_7
#define Fan1_PWM__GPIO_Port GPIOA
#define Fan2_Tach_Pin GPIO_PIN_0
#define Fan2_Tach_GPIO_Port GPIOB
#define Fan2_Tach_EXTI_IRQn EXTI0_1_IRQn
#define Fan2_PWM_Pin GPIO_PIN_1
#define Fan2_PWM_GPIO_Port GPIOB
#define Fan3_Tach_Pin GPIO_PIN_10
#define Fan3_Tach_GPIO_Port GPIOB
#define Fan3_Tach_EXTI_IRQn EXTI4_15_IRQn
#define Fan3_PWM_Pin GPIO_PIN_11
#define Fan3_PWM_GPIO_Port GPIOB
#define Pump_Tach_Pin GPIO_PIN_14
#define Pump_Tach_GPIO_Port GPIOB
#define Pump_Tach_EXTI_IRQn EXTI4_15_IRQn
#define Pump_PWM_Pin GPIO_PIN_15
#define Pump_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
