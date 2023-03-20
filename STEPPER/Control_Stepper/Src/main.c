e/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "stdio.h"
#include "Accel_stepper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*Declaire acceleraation for each stepper*/
Acceleration_t Stepper1;
Acceleration_t Stepper2;
//Acceleration_t Stepper3;
//Acceleration_t Stepper4;
//Acceleration_t Stepper5;
//Acceleration_t Stepper6;

/*
.
.
Acceleration_t Stepper#n
*/

/* USER CODE END PTD */
//Acceleration_t Stepper1;
//Acceleration_t Stepper2;
//Acceleration_t Stepper3;
//Acceleration_t Stepper4;
//Acceleration_t Stepper5;
//Acceleration_t Stepper6;

Inv_Arm_t Arm;
Arm_pos_t Arm_pos;
Arm_pos_t current_pos;
Memory_t memory;
Memory_t rxmemory;


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define max(a,b) (a > b) ? a : b;
#define min(a,b) (a > b) ? b : a;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;
float theta4 = 0;
float theta5 = 0;
float theta6 = 0;
int set_theta1 = 0;
int set_theta2 = 0;
//int set_theta3 = 0;
//int set_theta4 = 0;
//int set_theta5 = 0;
//int set_theta6 = 0;
//char uart_msg[100];
//char uart_buff[100];
//char msg_cmd[20];
//extern char rx_data;
//uint8_t freerun = 0;
//uint8_t input_mode = 0;
//uint8_t Lsw1 = 1;
//uint8_t Lsw2 = 1;
//uint8_t Lsw3 = 1;
//uint8_t Lsw4 = 1;
//uint8_t Lsw5 = 1;
//uint8_t Lsw6 = 1;
uint16_t theta1dot = 0;
uint16_t theta2dot = 0;
//uint16_t theta3dot = 0;
//uint16_t theta4dot = 0;
//uint16_t theta5dot = 0;
//uint16_t theta6dot = 0;
uint16_t max_speed = 200;
uint16_t accel1 = 0;
uint16_t accel2 = 0;
//uint16_t accel3 = 0;
//uint16_t accel4 = 0;
//uint16_t accel5 = 0;
//uint16_t accel6 = 0;
uint16_t a_max = 10;
//uint16_t j_max = 20;
//uint16_t jerk1,jerk2,jerk3,jerk4,jerk5,jerk6;

uint32_t step1 = 0;
uint32_t step2 = 0;
//uint32_t step3 = 0;
//uint32_t step4 = 0;
//uint32_t step5 = 0;
//uint32_t step6 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void stepper_set_rpm (uint16_t rpm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DelayUs_step(uint32_t us)
{
	HAL_TIM_Base_Start_IT(&htim6);
	//(&htim7)->Instance->CNT = (0);
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(__HAL_TIM_GET_COUNTER(&htim6) < us);
	HAL_TIM_Base_Stop_IT(&htim6);
}
// * Note:Formula to find angle step.
signed int angle2step(float angle, int ratio){
	    return angle*SPR*ratio/360;
//  	or return angle*SPR*(1/ratio)/360   ;//20=2*gearratio
}
signed int rad2step(float rads, int gear){
//	rads*SPR*10/(2*pi)=rads/ALPHA
	return rads*gear/ALPHA;//20=2*gearratio
}
void Syn_Stepper(int step1, int step2){ //int step3, int step4, int step5, int step6){
	int step_max = max(abs(step1), abs(step2)); // abs(step3), abs(step4), abs(step5), abs(step6));
	float coef1 = fabs(step1)/step_max;
	float coef2 = fabs(step2)/step_max;
//	float coef3 = fabs(step3)/step_max;
//	float coef4 = fabs(step4)/step_max;
//	float coef5 = fabs(step5)/step_max;
//	float coef6 = fabs(step6)/step_max;
	theta1dot = max_speed * coef1;
	theta2dot = max_speed * coef2;
//	theta3dot = max_speed * coef3;
//	theta4dot = max_speed * coef4;
//	theta5dot = max_speed * coef5;
//	theta6dot = max_speed * coef6;
	accel1 = a_max * coef1;
	accel2 = a_max * coef2;
//	accel3 = a_max * coef3;
//	accel4 = a_max * coef4;
//	accel5 = a_max * coef5;
//	accel6 = a_max * coef6;

//	jerk1 = j_max * coef1;
//	jerk2 = j_max * coef2;
//	jerk3 = j_max * coef3;
//	jerk4 = j_max * coef4;
//	jerk5 = j_max * coef5;
//	jerk6 = j_max * coef6;

}

/* USER CODE END 0 */
void MoveToPos(float x, float y, float z){
	cal_theta(x, y, z);
	set_theta1 = rad2step(Arm.theta1 - theta1, 10);
	set_theta2 = rad2step(Arm.theta2 - theta2, 20);
//	set_theta3 = -1.0* rad2step(fabs(theta3 - Arm.theta3 -(Arm.theta2 - theta2)), 20);//in quadrant IV
//	set_theta4 = rad2step(Arm.theta4 - theta4, 14);
	int step_max = max(abs(set_theta1), abs(set_theta2)); //abs(set_theta3),abs(set_theta4), abs(set_theta5), abs(set_theta6));
	float coef1 = fabs(set_theta1)/step_max;
	float coef2 = fabs(set_theta2)/step_max;
//	float coef3 = fabs(set_theta3)/step_max;
//	float coef4 = fabs(set_theta4)/step_max;
//	float coef5 = fabs(set_theta4)/step_max;
//	float coef6 = fabs(set_theta6)/step_max;
	theta1dot = max_speed * coef1;
	theta2dot = max_speed * coef2;
//	theta3dot = max_speed * coef3;
//	theta4dot = max_speed * coef4;
//	theta5dot = max_speed * coef5;
//	theta6dot = max_speed * coef6;
	accel1 = a_max * coef1;
	accel2 = a_max * coef2;
//	accel3 = a_max * coef3;
//	accel4 = a_max * coef4;
//	accel5 = a_max * coef5;
//	accel6 = a_max * coef6;
}
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
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
//  stepper_motor_set_param(MOTOR_X, 100);
//	  HAL_TIM_Base_Start_IT(&htim14);
	  Accel_Stepper_SetPin(&Stepper1, step_1_GPIO_Port, step_1_Pin, dir_1_GPIO_Port, dir_1_Pin);
	  Accel_Stepper_SetPin(&Stepper2, step_2_GPIO_Port, step_2_Pin, dir_2_GPIO_Port, dir_2_Pin);
//	  Accel_Stepper_SetPin(&Stepper3, step_3_GPIO_Port, step_3_Pin, dir_3_GPIO_Port, dir_3_Pin);
//	  Accel_Stepper_SetPin(&Stepper4, step_4_GPIO_Port, step_4_Pin, dir_4_GPIO_Port, dir_4_Pin);
//	  Accel_Stepper_SetPin(&Stepper5, step_5_GPIO_Port, step_5_Pin, dir_5_GPIO_Port, dir_5_Pin);
//	  Accel_Stepper_SetPin(&Stepper6, step_6_GPIO_Port, step_6_Pin, dir_6_GPIO_Port, dir_6_Pin);
	  Accel_Stepper_SetTimer(&Stepper1, &htim6);
	  Accel_Stepper_SetTimer(&Stepper2, &htim7);
//	  Accel_Stepper_SetTimer(&Stepper3, &htim10);
//	  Accel_Stepper_SetTimer(&Stepper4, &htim11);
//	  Accel_Stepper_SetTimer(&Stepper5, &htim12);
//	  Accel_Stepper_SetTimer(&Stepper6, &htim13);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  step1 = angle2step(360,14);
	  step2 = angle2step(360,10);
	  Syn_Stepper(step1, step2);
	  if(Stepper1.run_status == 0){ // run_status ==0 it mean we can change value when motor stop working
		  Accel_Stepper_Move(&Stepper1, step1 , accel1, accel1, theta1dot); //acell= decall
		                               //angle2step(angle,1 ratio of gearbox motor)
		                              //(Thelta  Accel   Deccel Speed)
//		  set_theta1 = 0;//reset steps to 0 (prevent re-run after done)
	  }

	if(Stepper2.run_status == 0){
		Accel_Stepper_Move(&Stepper2, step2 , accel2, accel2, theta2dot);

	}
	HAL_Delay(100);
//	x = angle2step(360, 1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	  if(htim->Instance == TIM6){//stepper1
		  Accel_Stepper_TIMIT_Handler(&Stepper1);
	  }
	  if(htim->Instance == TIM7){//stepper2
		  Accel_Stepper_TIMIT_Handler(&Stepper2);

	  }
}


//static unsigned long my_sqrt(unsigned long x)
//{
//  register unsigned long xr;  // result register
//  register unsigned long q2;  // scan-bit register
//  register unsigned char f;   // flag (one bit)
//
//  xr = 0;                     // clear result
//  q2 = 0x40000000L;           // higest possible result bit
//  do
//  {
//    if((xr + q2) <= x)
//    {
//      x -= xr + q2;
//      f = 1;                  // set flag
//    }
//    else{
//      f = 0;                  // clear flag
//    }
//    xr >>= 1;
//    if(f){
//      xr += q2;               // test flag
//    }
//  } while(q2 >>= 2);          // shift twice
//  if(xr < x){
//    return xr +1;             // add for rounding
//  }
//  else{
//    return xr;
//  }
//}

/*! \brief Find minimum value.
 *
 *  Returns the smallest value.
 *
 *  \return  Min(x,y).
 */
//unsigned int min(unsigned int x, unsigned int y)
//{
//  if(x < y){
//    return x;
//  }
//  else{
//    return y;
//  }
//}
/* USER CODE END 4 */

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
