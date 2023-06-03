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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Emergency_Pin GPIO_PIN_3
#define Emergency_GPIO_Port GPIOC
#define Emergency_EXTI_IRQn EXTI3_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Switch_Relay_3_Pin GPIO_PIN_5
#define Switch_Relay_3_GPIO_Port GPIOA
#define Switch_Relay_1_Pin GPIO_PIN_6
#define Switch_Relay_1_GPIO_Port GPIOA
#define Switch_Relay_2_Pin GPIO_PIN_7
#define Switch_Relay_2_GPIO_Port GPIOA
#define Photoelectric_sensor_2_Pin GPIO_PIN_2
#define Photoelectric_sensor_2_GPIO_Port GPIOB
#define Photoelectric_sensor_2_EXTI_IRQn EXTI2_IRQn
#define Photoelectric_sensor_1_Pin GPIO_PIN_6
#define Photoelectric_sensor_1_GPIO_Port GPIOC
#define Photoelectric_sensor_1_EXTI_IRQn EXTI9_5_IRQn
#define DIR_Pin GPIO_PIN_10
#define DIR_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define JoyStick_SS_PIN_Pin GPIO_PIN_2
#define JoyStick_SS_PIN_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Photoelectric_sensor_3_Pin GPIO_PIN_8
#define Photoelectric_sensor_3_GPIO_Port GPIOB
#define Photoelectric_sensor_3_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
