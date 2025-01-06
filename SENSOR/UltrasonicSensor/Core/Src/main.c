/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "stdio.h"
#include <math.h>

#include "hx711.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILTER_SIZE 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//    GPIO_TypeDef *ports[] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA, GPIOA, GPIOA};
//    uint16_t pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7,
//                       GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_4, GPIO_PIN_5};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t micros(void) {
    register uint32_t ms, cycle_cnt;

    // Ensure atomic reading of HAL_GetTick() and the timer count
    do {
        ms = HAL_GetTick();  // Millisecond tick count
        cycle_cnt = TIM11->CNT;  // Timer count for microseconds
    } while (ms != HAL_GetTick());  // Make sure ms has not changed

    // Return the time in microseconds
    return (ms * 1000) + cycle_cnt;
}

// @brief: Busy wait delay for a given number of microseconds (us)
void delay_us(uint32_t us) {
    uint32_t start = micros();  // Record the start time
    // Busy wait loop until the required delay has passed
    while ((micros() - start) < us) {
        asm volatile ("nop");  // No-operation instruction to prevent optimization
    }
}

// Trigger the ultrasonic sensor
void triggerUltrasonicSensor() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    delay_us(10);  // Limit rang
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    delay_us(10);  // Limit rang
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    delay_us(10);  // Limit rang
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    delay_us(10);  // Limit rang
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    delay_us(10);  // Limit rang
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    delay_us(10);  // Limit rang
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    delay_us(10);  // Limit rang
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    delay_us(10);  // Limit rang
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    delay_us(10);
}

//void triggerUltrasonicSensor() {
//    GPIO_TypeDef *ports[] = {U1_Trig_GPIO_Port, U2_Trig_GPIO_Port,U3_Trig_GPIO_Port, U4_Trig_GPIO_Port,
//    		                 U5_Trig_GPIO_Port, U6_Trig_GPIO_Port, U7_Trig_GPIO_Port, U8_Trig_GPIO_Port};
//    uint16_t pins[] = {U1_Trig_Pin, U2_Trig_Pin, U3_Trig_Pin, U4_Trig_Pin,
//    		           U5_Trig_Pin, U6_Trig_Pin, U7_Trig_Pin, U8_Trig_Pin};
//
//    for (int i = 0; i < 8; i++) {
//        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
//        delay_us(10);  // Trigger pulse width
//        HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
//        delay_us(10);  // Delay between triggers
//    }
//}

static uint32_t startTimes[8] = {0};
static uint32_t endTimes[8] = {0};
static float distances[8] = {0};

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

    if (htim->Instance == TIM1) { // Check the timer instance
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) { // Rising edge
                startTimes[0] = __HAL_TIM_GET_COUNTER(htim);
            } else { // Falling edge
                endTimes[0] = __HAL_TIM_GET_COUNTER(htim);
                uint32_t pulseWidth = endTimes[0] - startTimes[0];
                distances[0] = (pulseWidth * 0.0343) / 2; // Calculate distance
            }
        }if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) {
                startTimes[1] = __HAL_TIM_GET_COUNTER(htim);
            } else {
                endTimes[1] = __HAL_TIM_GET_COUNTER(htim);
                uint32_t pulseWidth = endTimes[1] - startTimes[1];
                distances[1] = (pulseWidth * 0.0343) / 2;
            }
        }if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) {
                startTimes[2] = __HAL_TIM_GET_COUNTER(htim);
            } else {
                endTimes[2] = __HAL_TIM_GET_COUNTER(htim);
                uint32_t pulseWidth = endTimes[2] - startTimes[2];
                distances[2] = (pulseWidth * 0.0343) / 2;
            }
        }if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) {
                startTimes[3] = __HAL_TIM_GET_COUNTER(htim);
            } else {
                endTimes[3] = __HAL_TIM_GET_COUNTER(htim);
                uint32_t pulseWidth = endTimes[3] - startTimes[3];
                distances[3] = (pulseWidth * 0.0343) / 2;
            }
        }
    }else if (htim->Instance == TIM2) { // Check the timer instance
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) { // Rising edge
                startTimes[4] = __HAL_TIM_GET_COUNTER(htim);
            } else { // Falling edge
                endTimes[4] = __HAL_TIM_GET_COUNTER(htim);
                uint32_t pulseWidth = endTimes[4] - startTimes[4];
                distances[4] = (pulseWidth * 0.0343) / 2; // Calculate distance
            }
        }if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) {
                startTimes[5] = __HAL_TIM_GET_COUNTER(htim);
            } else {
                endTimes[5] = __HAL_TIM_GET_COUNTER(htim);
                uint32_t pulseWidth = endTimes[5] - startTimes[5];
                distances[5] = (pulseWidth * 0.0343) / 2;
            }
        }if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) {
                startTimes[6] = __HAL_TIM_GET_COUNTER(htim);
            } else {
                endTimes[6] = __HAL_TIM_GET_COUNTER(htim);
                uint32_t pulseWidth = endTimes[6] - startTimes[6];
                distances[6] = (pulseWidth * 0.0343) / 2;
            }
        }if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
                startTimes[7] = __HAL_TIM_GET_COUNTER(htim);
            } else {
                endTimes[7] = __HAL_TIM_GET_COUNTER(htim);
                uint32_t pulseWidth = endTimes[7] - startTimes[7];
                distances[7] = (pulseWidth * 0.0343) / 2;
            }
        }
    }
}


float distanceBuffer[FILTER_SIZE] = {0};
uint8_t bufferIndex = 0;

float applyMovingAverage(float newDistance){
    distanceBuffer[bufferIndex] = newDistance;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += distanceBuffer[i];
    }

    return sum / FILTER_SIZE;
}


HX711 hx711;
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//    // Array of timer instances and channels
//    TIM_TypeDef *timInstances[] = {TIM1, TIM2};
//    GPIO_TypeDef *gpioPorts[] = {U1_Echo_GPIO_Port,U2_Echo_GPIO_Port, U3_Echo_GPIO_Port, U4_Echo_GPIO_Port,
//    		                     U5_Echo_GPIO_Port, U6_Echo_GPIO_Port, U7_Echo_GPIO_Port, U8_Echo_GPIO_Port};
//    uint16_t gpioPins[] = {U1_Echo_Pin, U2_Echo_Pin, U3_Echo_Pin, U4_Echo_Pin,
//    		               U5_Echo_Pin, U6_Echo_Pin, U7_Echo_Pin, U8_Echo_Pin};
//
//    for (int i = 0; i < 2; i++) { // Loop over TIM1 and TIM2
//        if (htim->Instance == timInstances[i]) {
//            for (int ch = 0; ch < 4; ch++) { // Loop over channels 1-4
//                if (htim->Channel == (HAL_TIM_ACTIVE_CHANNEL_1 + ch)) {
//                    int idx = (i * 4) + ch; // Calculate array index
//                    if (HAL_GPIO_ReadPin(gpioPorts[idx], gpioPins[idx]) == GPIO_PIN_SET) { // Rising edge
//                        startTimes[idx] = __HAL_TIM_GET_COUNTER(htim);
//                    } else { // Falling edge
//                        endTimes[idx] = __HAL_TIM_GET_COUNTER(htim);
//                        uint32_t pulseWidth = endTimes[idx] - startTimes[idx];
//                        distances[idx] = (pulseWidth * 0.0343) / 2; // Calculate distance
//                    }
//                }
//            }
//        }
//    }
//}

//void InitTimers() {
//    TIM_HandleTypeDef *timers[] = {&htim1, &htim2}; // Array of timer handles
//
//    for (int i = 0; i < 2; i++) { // Loop over timers
//        for (int ch = TIM_CHANNEL_1; ch <= TIM_CHANNEL_4; ch += 1) { // Loop over channels 1-4
//            HAL_TIM_IC_Start_IT(timers[i], ch); // Start Input Capture interrupt
//        }
//    }
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();


  // Setup HX711
//  hx711.gpioSck = GPIOB;
//  hx711.gpioData = GPIOB;
//  hx711.pinSck = GPIO_PIN_13;
//  hx711.pinData = GPIO_PIN_12;
//  hx711.gain = 1;
//
//  HX711_Init(hx711);
//
//  hx711 = HX711_Tare(hx711, 10);
  /* USER CODE BEGIN 2 */

//  InitTimers();

 HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
 HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
 HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
 HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

 HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
 HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
 HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
 HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//      int average_value = HX711_Average_Value(hx711, 5);
//      int net_value = average_value - hx711.offset;
//      float weight_in_grams = net_value / SCALE_FACTOR;
//
//      // Output weight
//      printf("Weight: %.2f grams\n", weight_in_grams); // @suppress("Float formatting support")
//
//      HAL_Delay(500);

	  triggerUltrasonicSensor();
//	  applyMovingAverage();
	  HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
