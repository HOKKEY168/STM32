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
#define U5_Echo_Pin GPIO_PIN_0
#define U5_Echo_GPIO_Port GPIOA
#define U5_Trig_Pin GPIO_PIN_1
#define U5_Trig_GPIO_Port GPIOA
#define U6_Trig_Pin GPIO_PIN_2
#define U6_Trig_GPIO_Port GPIOA
#define U8_Echo_Pin GPIO_PIN_3
#define U8_Echo_GPIO_Port GPIOA
#define U7_Trig_Pin GPIO_PIN_4
#define U7_Trig_GPIO_Port GPIOA
#define U8_Trig_Pin GPIO_PIN_5
#define U8_Trig_GPIO_Port GPIOA
#define U7_Echo_Pin GPIO_PIN_10
#define U7_Echo_GPIO_Port GPIOB
#define U1_Echo_Pin GPIO_PIN_8
#define U1_Echo_GPIO_Port GPIOA
#define U2_Echo_Pin GPIO_PIN_9
#define U2_Echo_GPIO_Port GPIOA
#define U3_Echo_Pin GPIO_PIN_10
#define U3_Echo_GPIO_Port GPIOA
#define U4_Echo_Pin GPIO_PIN_11
#define U4_Echo_GPIO_Port GPIOA
#define U6_Echo_Pin GPIO_PIN_3
#define U6_Echo_GPIO_Port GPIOB
#define U1_Trig_Pin GPIO_PIN_4
#define U1_Trig_GPIO_Port GPIOB
#define U2_Trig_Pin GPIO_PIN_5
#define U2_Trig_GPIO_Port GPIOB
#define U3_Trig_Pin GPIO_PIN_6
#define U3_Trig_GPIO_Port GPIOB
#define U4_Trig_Pin GPIO_PIN_7
#define U4_Trig_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
