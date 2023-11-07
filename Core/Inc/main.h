/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define Error_LED_Pin GPIO_PIN_0
#define Error_LED_GPIO_Port GPIOA
#define STM_OK_Pin GPIO_PIN_1
#define STM_OK_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_4
#define CS1_GPIO_Port GPIOC
#define CS0_Pin GPIO_PIN_5
#define CS0_GPIO_Port GPIOC
#define HeadLights_Pin GPIO_PIN_0
#define HeadLights_GPIO_Port GPIOB
#define RightTurn_Pin GPIO_PIN_1
#define RightTurn_GPIO_Port GPIOB
#define LeftTurn_Pin GPIO_PIN_2
#define LeftTurn_GPIO_Port GPIOB
#define ARRAY_EN_Pin GPIO_PIN_14
#define ARRAY_EN_GPIO_Port GPIOB
#define BPS_EN_Pin GPIO_PIN_15
#define BPS_EN_GPIO_Port GPIOB
#define MC_EN_Pin GPIO_PIN_6
#define MC_EN_GPIO_Port GPIOC
#define ARRAY_LED_Pin GPIO_PIN_7
#define ARRAY_LED_GPIO_Port GPIOC
#define BPS_ENC8_Pin GPIO_PIN_8
#define BPS_ENC8_GPIO_Port GPIOC
#define MC_ENC9_Pin GPIO_PIN_9
#define MC_ENC9_GPIO_Port GPIOC
#define E2_Pin GPIO_PIN_10
#define E2_GPIO_Port GPIOC
#define E1_Pin GPIO_PIN_11
#define E1_GPIO_Port GPIOC
#define E0_Pin GPIO_PIN_12
#define E0_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
