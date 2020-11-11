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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BT_UP_Pin GPIO_PIN_6
#define BT_UP_GPIO_Port GPIOA
#define BT_UP_EXTI_IRQn EXTI9_5_IRQn
#define BT_CENTER_Pin GPIO_PIN_7
#define BT_CENTER_GPIO_Port GPIOA
#define BT_CENTER_EXTI_IRQn EXTI9_5_IRQn
#define BT_DOWN_Pin GPIO_PIN_8
#define BT_DOWN_GPIO_Port GPIOA
#define BT_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define LED_DEBUG_Pin GPIO_PIN_11
#define LED_DEBUG_GPIO_Port GPIOA
#define RELAY_CONTROL_PIN_Pin GPIO_PIN_12
#define RELAY_CONTROL_PIN_GPIO_Port GPIOA
#define INT4_NO_PWM_MOTOR_2_Pin GPIO_PIN_15
#define INT4_NO_PWM_MOTOR_2_GPIO_Port GPIOA
#define INT3_PWM_TIM3_CH2_MOTOR_2_Pin GPIO_PIN_5
#define INT3_PWM_TIM3_CH2_MOTOR_2_GPIO_Port GPIOB
#define INT2_NO_PWM_MOTOR_1_Pin GPIO_PIN_7
#define INT2_NO_PWM_MOTOR_1_GPIO_Port GPIOB
#define INT1_PWM_TIM4_CH3_MOTOR_1_Pin GPIO_PIN_8
#define INT1_PWM_TIM4_CH3_MOTOR_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
