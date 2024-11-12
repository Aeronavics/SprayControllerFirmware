/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
void SystemClock_Config(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRST_Pin GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
#define FLOW_4_Pin GPIO_PIN_0
#define FLOW_4_GPIO_Port GPIOC
#define FLOW_4_EXTI_IRQn EXTI0_IRQn
#define FLOW_3_Pin GPIO_PIN_1
#define FLOW_3_GPIO_Port GPIOC
#define FLOW_3_EXTI_IRQn EXTI1_IRQn
#define FLOW_2_Pin GPIO_PIN_2
#define FLOW_2_GPIO_Port GPIOC
#define FLOW_2_EXTI_IRQn EXTI2_IRQn
#define FLOW_1_Pin GPIO_PIN_3
#define FLOW_1_GPIO_Port GPIOC
#define FLOW_1_EXTI_IRQn EXTI3_IRQn
#define FAN_4_Pin GPIO_PIN_1
#define FAN_4_GPIO_Port GPIOA
#define FAN_3_Pin GPIO_PIN_2
#define FAN_3_GPIO_Port GPIOA
#define FAN_2_Pin GPIO_PIN_3
#define FAN_2_GPIO_Port GPIOA
#define FAN_1_Pin GPIO_PIN_5
#define FAN_1_GPIO_Port GPIOA
#define ERROR0_Pin GPIO_PIN_6
#define ERROR0_GPIO_Port GPIOA
#define ERROR1_Pin GPIO_PIN_7
#define ERROR1_GPIO_Port GPIOA
#define STATUS1_Pin GPIO_PIN_0
#define STATUS1_GPIO_Port GPIOB
#define STATUS2_Pin GPIO_PIN_1
#define STATUS2_GPIO_Port GPIOB
#define STATUS3_Pin GPIO_PIN_2
#define STATUS3_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
