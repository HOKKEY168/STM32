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
#define ENC_M5_A_Pin GPIO_PIN_6
#define ENC_M5_A_GPIO_Port GPIOA
#define ENC_M5_A_EXTI_IRQn EXTI9_5_IRQn
#define ENC_M5_B_Pin GPIO_PIN_7
#define ENC_M5_B_GPIO_Port GPIOA
#define ENC_M5_B_EXTI_IRQn EXTI9_5_IRQn
#define ENC_M6_A_Pin GPIO_PIN_4
#define ENC_M6_A_GPIO_Port GPIOC
#define ENC_M6_A_EXTI_IRQn EXTI4_IRQn
#define ENC_M6_B_Pin GPIO_PIN_5
#define ENC_M6_B_GPIO_Port GPIOC
#define ENC_M6_B_EXTI_IRQn EXTI9_5_IRQn
#define step1_Pin GPIO_PIN_10
#define step1_GPIO_Port GPIOE
#define dir1_Pin GPIO_PIN_11
#define dir1_GPIO_Port GPIOE
#define step_Pin GPIO_PIN_12
#define step_GPIO_Port GPIOE
#define dir_Pin GPIO_PIN_13
#define dir_GPIO_Port GPIOE
#define ENC_M4_A_Pin GPIO_PIN_12
#define ENC_M4_A_GPIO_Port GPIOB
#define ENC_M4_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC_M4_B_Pin GPIO_PIN_13
#define ENC_M4_B_GPIO_Port GPIOB
#define ENC_M4_B_EXTI_IRQn EXTI15_10_IRQn
#define ENC_M3_A_Pin GPIO_PIN_14
#define ENC_M3_A_GPIO_Port GPIOB
#define ENC_M3_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC_M3_B_Pin GPIO_PIN_15
#define ENC_M3_B_GPIO_Port GPIOB
#define ENC_M3_B_EXTI_IRQn EXTI15_10_IRQn
#define ENC_M2_A_Pin GPIO_PIN_8
#define ENC_M2_A_GPIO_Port GPIOD
#define ENC_M2_A_EXTI_IRQn EXTI9_5_IRQn
#define ENC_M2_B_Pin GPIO_PIN_9
#define ENC_M2_B_GPIO_Port GPIOD
#define ENC_M2_B_EXTI_IRQn EXTI9_5_IRQn
#define ENC_M1_A_Pin GPIO_PIN_10
#define ENC_M1_A_GPIO_Port GPIOD
#define ENC_M1_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC_M1_B_Pin GPIO_PIN_11
#define ENC_M1_B_GPIO_Port GPIOD
#define ENC_M1_B_EXTI_IRQn EXTI15_10_IRQn
#define PWM_M4_R_Pin GPIO_PIN_6
#define PWM_M4_R_GPIO_Port GPIOC
#define PWM_M4_L_Pin GPIO_PIN_7
#define PWM_M4_L_GPIO_Port GPIOC
#define PWM_M3_R_Pin GPIO_PIN_8
#define PWM_M3_R_GPIO_Port GPIOC
#define PWM_M3_L_Pin GPIO_PIN_9
#define PWM_M3_L_GPIO_Port GPIOC
#define PWM_M2_R_Pin GPIO_PIN_8
#define PWM_M2_R_GPIO_Port GPIOA
#define PWM_M2_L_Pin GPIO_PIN_9
#define PWM_M2_L_GPIO_Port GPIOA
#define PWM_M1_R_Pin GPIO_PIN_10
#define PWM_M1_R_GPIO_Port GPIOA
#define PWM_M1_L_Pin GPIO_PIN_11
#define PWM_M1_L_GPIO_Port GPIOA
#define LS3_Pin GPIO_PIN_1
#define LS3_GPIO_Port GPIOD
#define LS3_EXTI_IRQn EXTI1_IRQn
#define LS1_Pin GPIO_PIN_2
#define LS1_GPIO_Port GPIOD
#define LS1_EXTI_IRQn EXTI2_IRQn
#define LS2_Pin GPIO_PIN_3
#define LS2_GPIO_Port GPIOD
#define LS2_EXTI_IRQn EXTI3_IRQn
#define CE_Pin GPIO_PIN_6
#define CE_GPIO_Port GPIOD
#define CSN_Pin GPIO_PIN_7
#define CSN_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
