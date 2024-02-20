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
#define TRIG_1_Pin GPIO_PIN_0
#define TRIG_1_GPIO_Port GPIOC
#define TRIG_2_Pin GPIO_PIN_1
#define TRIG_2_GPIO_Port GPIOC
#define TURN_OFF_B2_Pin GPIO_PIN_3
#define TURN_OFF_B2_GPIO_Port GPIOC
#define RTR_OUT_Pin GPIO_PIN_4
#define RTR_OUT_GPIO_Port GPIOA
#define TRIG_3_Pin GPIO_PIN_0
#define TRIG_3_GPIO_Port GPIOB
#define ATT_Pin GPIO_PIN_10
#define ATT_GPIO_Port GPIOB
#define GREEN_Pin GPIO_PIN_13
#define GREEN_GPIO_Port GPIOB
#define YELLOW_Pin GPIO_PIN_14
#define YELLOW_GPIO_Port GPIOB
#define RED_Pin GPIO_PIN_15
#define RED_GPIO_Port GPIOB
#define OSC_Pin GPIO_PIN_9
#define OSC_GPIO_Port GPIOA
#define ACK_IN_Pin GPIO_PIN_10
#define ACK_IN_GPIO_Port GPIOA
#define ACK_IN_EXTI_IRQn EXTI15_10_IRQn
#define ACK_OUT_Pin GPIO_PIN_5
#define ACK_OUT_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_8
#define S2_GPIO_Port GPIOB
#define RTR_IN_Pin GPIO_PIN_9
#define RTR_IN_GPIO_Port GPIOB
#define RTR_IN_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */