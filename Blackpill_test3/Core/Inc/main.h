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
#define Input_10_Pin GPIO_PIN_15
#define Input_10_GPIO_Port GPIOC
#define Input_0_Pin GPIO_PIN_0
#define Input_0_GPIO_Port GPIOA
#define Input_1_Pin GPIO_PIN_1
#define Input_1_GPIO_Port GPIOA
#define Input_2_Pin GPIO_PIN_2
#define Input_2_GPIO_Port GPIOA
#define Input_3_Pin GPIO_PIN_3
#define Input_3_GPIO_Port GPIOA
#define Input_4_Pin GPIO_PIN_4
#define Input_4_GPIO_Port GPIOA
#define Input_5_Pin GPIO_PIN_5
#define Input_5_GPIO_Port GPIOA
#define Input_6_Pin GPIO_PIN_6
#define Input_6_GPIO_Port GPIOA
#define Input_7_Pin GPIO_PIN_7
#define Input_7_GPIO_Port GPIOA
#define Input_8_Pin GPIO_PIN_0
#define Input_8_GPIO_Port GPIOB
#define Input_9_Pin GPIO_PIN_1
#define Input_9_GPIO_Port GPIOB
#define Input_11_Pin GPIO_PIN_10
#define Input_11_GPIO_Port GPIOB
#define Input_12_Pin GPIO_PIN_12
#define Input_12_GPIO_Port GPIOB
#define Input_13_Pin GPIO_PIN_13
#define Input_13_GPIO_Port GPIOB
#define Input_14_Pin GPIO_PIN_14
#define Input_14_GPIO_Port GPIOB
#define Input_15_Pin GPIO_PIN_15
#define Input_15_GPIO_Port GPIOB
#define EN_1_Pin GPIO_PIN_8
#define EN_1_GPIO_Port GPIOA
#define EN_2_Pin GPIO_PIN_9
#define EN_2_GPIO_Port GPIOA
#define PL_1_Pin GPIO_PIN_10
#define PL_1_GPIO_Port GPIOA
#define PL_2_Pin GPIO_PIN_11
#define PL_2_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_12
#define CLK_GPIO_Port GPIOA
#define Shift_data_out_Pin GPIO_PIN_4
#define Shift_data_out_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
