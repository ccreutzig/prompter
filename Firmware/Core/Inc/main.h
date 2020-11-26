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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Status_LED_Pin GPIO_PIN_2
#define Status_LED_GPIO_Port GPIOC
#define DMX_R_W_Pin GPIO_PIN_2
#define DMX_R_W_GPIO_Port GPIOB
#define Foot_R_Pin GPIO_PIN_13
#define Foot_R_GPIO_Port GPIOB
#define Foot_L_Pin GPIO_PIN_14
#define Foot_L_GPIO_Port GPIOB
#define L1MAN_Pin GPIO_PIN_6
#define L1MAN_GPIO_Port GPIOC
#define L2MAN_Pin GPIO_PIN_7
#define L2MAN_GPIO_Port GPIOC
#define L3MAN_Pin GPIO_PIN_8
#define L3MAN_GPIO_Port GPIOC
#define L4MAN_Pin GPIO_PIN_9
#define L4MAN_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOB
#define PWM3_Pin GPIO_PIN_8
#define PWM3_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_9
#define PWM4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
