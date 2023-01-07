/* USER CODE BEGIN Header */
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_NRF24.h"
#include "KinematicEquation.h"
#include "EncoderMotor.h"
#include "PID.h"
#include "DefinePID.h"
#include "Accel_stepper.h"
#include "Defined.h"
#include "pca9685.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PIDController PIDconfig;
//ENCODER_t ENC_Ro;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * some defined for nrf24
 */

uint64_t RxpipeAddrs = 0x11223344AA;
uint8_t RxData[32];
uint8_t myAckPayload[32];

/*
 * some defined for base wheel
 */

uint32_t Enc_count[6] = { 0 };
uint8_t dir[6] = { 0 };
uint8_t nowA[6] = { 0 };
uint8_t nowB[6] = { 0 };
uint16_t cnt[6] = { 0 };
uint8_t lastA[6] = { 0 };
uint8_t lastB[6] = { 0 };

float Motor1_speed = 0;
float Motor2_speed = 0;
float Motor3_speed = 0;
float Motor4_speed = 0;
float Motor5_speed = 0;

float V1 = 0; //target speed of motor1
float V2 = 0; //target speed of motor2
float V3 = 0; //target speed of motor3
float V4 = 0; //target speed of motor4
float V5 = 0; //target speed of motor5/shooting motor

float V_set = 0; // for shooting speed
float V_set1 = 0;

float pwm_M1 = 0;
float pwm_M2 = 0;
float pwm_M3 = 0;
float pwm_M4 = 0;

/*
 * initial pwm val for shooting motor
 */
float pwm_M5 = 0;

float vx = 0;
float vx1 = 0;
float vy = 0;
float vy1 = 0;
float omega = 0;
float omega1 = 0;

float rdps[5] = { 0 };
float kmph[5] = { 0 };

uint32_t SampleTime = 0; //speed sample time for Motor(millisecond)
uint32_t lastTime = 0;

int cnt1 = 0;
Acceleration_t Stepper1;
//Acceleration_t Stepper2;

int stepperConFor_Tie = 0;
int stepperConBac_Tie = 0;
int stepperConFor_Z = 0;
int stepperConBac_Z = 0;
int teta_plus = 0;
int teta_minus = 0;

/*
 * for Enable shooting status
 */
int countStatus = 0;
int state = 0;
int pre_state = 0;

int slideFor = 0;
int slideBac = 0;
int slideCount = 0;

int RotateFor = 0;
int RotateBac = 0;

int ls1_state = 1;
int ls2_state = 1;
int ls3_state = 0;
int ls0_state = 0;

int Speed = 115;

#define true 1
#define false 0

int state10 = 0;
int state11 = 0;

int gribBac = 0;
int h = 0;

int S_servo1 = 0;
int S_servo2 = 0;
int S_servo3 = 0;

//float pwm1;pwm2;pwm3;pwm4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float map(float Input, float Min_Input, float Max_Input, float Min_Output,
		float Max_Output) {
	return (float) ((Input - Min_Input) * (Max_Output - Min_Output)
			/ (Max_Input - Min_Input) + Min_Output);
}

void reset_data(void) {
	RxData[0] = 128; //x1
	RxData[1] = 128; //y1
	RxData[2] = 128; //x2
	RxData[3] = 128; //y2

	for (int i = 5; i < 32; i++) {
		RxData[i] = 0;
	}
}

void stepperCon(void) {
	htim13.Instance->ARR = Speed;
}

/*
 * state for checking condition/ Enable shooting
 */
void status(void) {

	if (RxData[7] == 32) {
		state = 1;
	} else {
		state = 0;
	}
	if (state != pre_state) {
		if (state == 1) {
			countStatus++;
		}
	}
	pre_state = state;
	if (countStatus >= 3) {
		countStatus = 0;
	}

}

void gripperSethome(void) {
	while (ls3_state != 1) {
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 350);

	}

	stepperConBac_Tie = 0;
	stepperConFor_Tie = 0;
	Accel_Stepper_Move(&Stepper1, 1600 * 2, 500, 500, 1000);
	gribBac = 1;

	if (gribBac == 1) {
		while (cnt[5] <= 350) {
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 700);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
		}
	}

}

void slideStatus(void) {
	if (RxData[7] == 64)
		slideFor = 1;
	if (RxData[7] != 64)
		slideFor = 0;

	if (RxData[7] == 128)
		RotateFor = 1;
	if (RxData[7] != 128)
		RotateFor = 0;

	if (RxData[7] == 16)
		RotateBac = 1;
	if (RxData[7] != 16)
		RotateBac = 0;

	if (RxData[7] == 1)
		teta_plus = 1;
	if (RxData[7] != 1)
		teta_plus = 0;

	if (RxData[7] == 2)
		teta_minus = 1;
	if (RxData[7] != 2)
		teta_minus = 0;
}

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_TIM8_Init();
  MX_TIM14_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM13_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	/*.............................Defined NRF24..............................................*/
	NRF24_begin(CSN_GPIO_Port, CSN_Pin, CE_Pin, hspi1);
	NRF24_setAutoAck(true);
	NRF24_setChannel(52);
	NRF24_setPayloadSize(32);
	NRF24_startListening();
	NRF24_openReadingPipe(1, RxpipeAddrs);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();

	reset_data();
	for (int i = 0; i < numberofMotor; i++) {
		count[i] = 0;
	}
	/*------------------------------------Initial Encoder for Rotate Motor---------------------*/
//	Encoder_Init(&ENC_Ro);
	/*----------------------------------Timer interrupt for sampling time 10ms or 100Hz-------*/
	HAL_TIM_Base_Start_IT(&htim10); //timer interrupt for sampling time 10ms or 100Hz
	HAL_TIM_Base_Start_IT(&htim13); //timer interrupt for stepper

	/*----------------------------------Start PWM for motor Driver---------------------*/
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

	PID_Init(&PIDconfig, numberofMotor);
	PIDconfig.tau = 0.005; //time constant for low pass filter
	/* Output limits */
	PIDconfig.limMin = -990;
	PIDconfig.limMax = 990;
	/* Integrator limits */
	PIDconfig.limMinInt = -700;
	PIDconfig.limMaxInt = 700;
	/* Sample time (in seconds) */
	PIDconfig.T = 0.01;
	/*
	 * starting initial stepper library for stepper 1
	 */
//
//	Accel_Stepper_SetPin(&Stepper1, step_GPIO_Port, step_Pin, dir_GPIO_Port,
//			dir_Pin);
//	Accel_Stepper_SetTimer(&Stepper1, &htim14);

	PCA9685_Init(&hi2c1);
	PCA9685_SetServoAngle(14, 130);//30
	PCA9685_SetServoAngle(15, 30);//130
	HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*
		 * shooting condition
		 */

		myAckPayload[0] = 1;

		if (NRF24_available()) {
			NRF24_read(RxData, 32);

			NRF24_writeAckPayload(1, myAckPayload, 32);

			lastTime = HAL_GetTick();
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
		}

		if ((HAL_GetTick() - lastTime) > 10) {
			/*
			 * *10 here is 10ms
			 */
			reset_data();
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
		}

		vx1    = (float) RxData[1];
		vy1    = (float) RxData[0];
		omega1 = (float) RxData[2];
		V_set1 = (float) RxData[4];

		if (vx1 > 120 && vx1 < 135) {
			vx1 = 128;
		}
		if (vy1 > 120 && vy1 < 135) {
			vy1 = 128;
		}
		if (omega1 > 120 && omega1 < 135) {
			omega1 = 128;
		}

		V_set = map(V_set1, 0, 256, 8, 135);
		vx = map(vy1, 0, 256, 380, -380);
		vy = map(vx1, 0, 256, 380, -380);
		omega = map(omega1, 0, 256, -10, 10);

		/*
		 * stepper
		 */

		stepperCon();
		if (RxData[8] == 1)stepperConFor_Tie = true;
		if (RxData[8] != 1) stepperConFor_Tie = false;
		if (RxData[8] == 4) stepperConBac_Tie = true;
		if (RxData[8] != 4) stepperConBac_Tie = false;

		/*
		 * for axe z rotate
		 */

		if (RxData[7] == 4) stepperConFor_Z = true;
		if (RxData[7] != 4) stepperConFor_Z = false;
		if (RxData[7] == 8) stepperConBac_Z = true;
		if (RxData[7] != 8) stepperConBac_Z = false;
//		if (stepperConBac == 1) {
//		Accel_Stepper_Move(&Stepper1, -64000, 2000, 2000, 10000);
//		}

//		if (RxData[7]==)
		/*
		 * slider
		 */

		if (slideFor == 1 && state10 == 1 && state11 == 0) {
			slideCount = 1;
		}
		/*
		 * slider
		 */
		if (slideCount == 1) {
			while (state10 == 1 && state11 == 0) {
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 700);
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
			}
			slideBac = 1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

		}
		if (slideBac == 1) {
			while (state11 == 1 && state10 == 0) {
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 300);
			}
			slideFor = 0;
			slideCount = 0;
			TIM5->CCR3=0;
			TIM5->CCR4=0;
		}

		if (teta_plus == 1) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 900);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

		} else if (teta_minus == 1) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 900);

		} else {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

		}

		/*
		 * Condition for Servo
		 */

		if (RxData[7] == 32) {
			if (S_servo3 == 1) {
				PCA9685_SetServoAngle(14, 130);//30
				HAL_Delay(10);
				PCA9685_SetServoAngle(15, 30);//130
				S_servo3 = 0;

			}
			if (S_servo2 == 1) {
				PCA9685_SetServoAngle(14, 30);//130
				S_servo2 = 0;
			}
			if (S_servo1 == 1) {
				PCA9685_SetServoAngle(15, 130);//30
				S_servo1 = 0;
			}

		}

		/*
		 * Rotate Part
		 */

		if (RotateFor == 1) {

			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 900);

		} else if (RotateBac == 1) {
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 900);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

		} else {
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);

		}

//       Stepper1.run_state =STOP;

		/*
		 * for stopping Stepper
		 */
//		if(Accel[STEPPER1].run_state ==STOP){
//
//		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM14) {
		Accel_Stepper_TIMIT_Handler(&Stepper1);
//		Accel_Stepper_TIMIT_Handler(&Stepper2);
	}

	if (htim->Instance == TIM13) {
		if (stepperConBac_Tie == 1) {

			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
		}
		if (stepperConFor_Tie == 1) {

			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
		}
		if (stepperConBac_Z == 1) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
		}
		if (stepperConFor_Z == 1) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
		}
	}

	if (htim->Instance == TIM10) {

		slideStatus();
		status();

		if (ls2_state == 0) {
			state10 = true;
			state11 = false;
		}
		if (ls1_state == 0) {
			state11 = true;
			state10 = false;
		}

		if (countStatus == 0) S_servo1 = 1;
		if (countStatus != 0) S_servo1 = 0;
		if (countStatus == 1) S_servo2 = 1;
		if (countStatus != 1) S_servo2 = 0;
		if (countStatus == 2) S_servo3 = 1;
		if (countStatus != 2) S_servo3 = 0;

		V1 = KinematicM1(vx, vy, omega);
		V2 = KinematicM2(vx, vy, omega);
		V3 = KinematicM3(vx, vy, omega);
		V4 = KinematicM4(vx, vy, omega);
		V5 = V_set / (0.1885 * 0.11 * 9.554);
		/*------------------------get the PWM form PID------------------------------*/
		/*
		 * pwm_M = PID(&myPID, Set point, measurement);
		 */
		pwm_M1 = PID(&PIDconfig, V1, Motor1_speed, Kp, Ki, Kd, 1);
		pwm_M2 = PID(&PIDconfig, V2, Motor2_speed, Kp, Ki, Kd, 2);
		pwm_M3 = PID(&PIDconfig, V3, Motor3_speed, Kp, Ki, Kd, 3);
		pwm_M4 = PID(&PIDconfig, V4, Motor4_speed, Kp, Ki, Kd, 4);
		/*
		 * PID for shooting motor
		 */
		pwm_M5 = PID(&PIDconfig, V5, Motor5_speed, 13, 325, 0, 5);
		/*
		 * PID for Rotate motor
		 */
//		pwm_M6 = PID(&PIDconfig, V6, Motor6_speed, 13, 325, 0, 5):
		/*
		 * Motor_speed= Motor_Speed(define Motor, sampling Time, encoder 1 round);
		 */
		Motor1_speed = Motor_Speed(Motor1, 10, 925);
		Motor2_speed = Motor_Speed(Motor2, 10, 925);
		Motor3_speed = Motor_Speed(Motor3, 10, 925);
		Motor4_speed = Motor_Speed(Motor4, 10, 925);
		/*
		 * motor speed for shooting motor
		 */
		Motor5_speed = Motor_Speed(Motor5, 10, 55);
		/*
		 * motor speed for rotating motor
		 */
//		Motor6_speed = Motor_Speed(6,10,55);
		/*
		 * Provide PWM for Motor1
		 */
		if (pwm_M1 > 0) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_M1); //CW

		} else if (pwm_M1 < 0) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (-1.0) * pwm_M1);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

		} else {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

		}
		/*
		 * provide PWM for motor2 5
		 */
		if (pwm_M2 > 0) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_M2);
		} else if (pwm_M2 < 0) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (-1.0) * pwm_M2);
		} else {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		}
		/*
		 * Provide PWM for motor3
		 */
		if (pwm_M3 > 0) {
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm_M3);
		} else if (pwm_M3 < 0) {
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (-1.0) * pwm_M3);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
		} else {
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
		}
		/*
		 * Provide PWM for motor4
		 */
		if (pwm_M4 > 0) {
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm_M4);
		} else if (pwm_M4 < 0) {
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (-1.0) * pwm_M4);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
		} else {
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
		}
		/*
		 * provide PWM for motor shooting
		 */

		if (pwm_M5 > 0 && RxData[6] == 1) { // shooting motor
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_M5);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_M5);

		} else if (pwm_M5 < 0) {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		} else {
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

		}



	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	/*
	 * for reading signal from every encoder of motors
	 */

	if (GPIO_Pin == ENC_M1_A_Pin || ENC_M1_B_Pin) {
		nowA[0] = HAL_GPIO_ReadPin(ENC_M1_A_GPIO_Port, ENC_M1_A_Pin);
		nowB[0] = HAL_GPIO_ReadPin(ENC_M1_B_GPIO_Port, ENC_M1_B_Pin);
		Enc_count[0] = encoder(0);
	}

	if (GPIO_Pin == ENC_M2_A_Pin || ENC_M2_B_Pin) { //ENCODER Motor 2
		nowA[1] = HAL_GPIO_ReadPin(ENC_M2_A_GPIO_Port, ENC_M2_A_Pin);
		nowB[1] = HAL_GPIO_ReadPin(ENC_M2_B_GPIO_Port, ENC_M2_B_Pin);
		Enc_count[1] = encoder(1);
	}

	if (GPIO_Pin == ENC_M3_A_Pin || ENC_M3_B_Pin) { //ENCODER Motor 3
		nowA[2] = HAL_GPIO_ReadPin(ENC_M3_A_GPIO_Port, ENC_M3_A_Pin);
		nowB[2] = HAL_GPIO_ReadPin(ENC_M3_B_GPIO_Port, ENC_M3_B_Pin);
		Enc_count[2] = encoder(2);
	}

	if (GPIO_Pin == ENC_M4_A_Pin || ENC_M4_B_Pin) { //ENCODER Motor 4
		nowA[3] = HAL_GPIO_ReadPin(ENC_M4_A_GPIO_Port, ENC_M4_A_Pin);
		nowB[3] = HAL_GPIO_ReadPin(ENC_M4_B_GPIO_Port, ENC_M4_B_Pin);
		Enc_count[3] = encoder(3);
	}

	if (GPIO_Pin == ENC_M5_A_Pin || ENC_M5_B_Pin) { // ENCODER Motor5
		nowA[4] = HAL_GPIO_ReadPin(ENC_M5_A_GPIO_Port, ENC_M5_A_Pin);
		nowB[4] = HAL_GPIO_ReadPin(ENC_M5_B_GPIO_Port, ENC_M5_B_Pin);
		Enc_count[4] = encoder(4);
	}

//	if (GPIO_Pin == ENC_M6_A_Pin || ENC_M6_B_Pin) { // ENCODER Motor6
//		nowA[5] = HAL_GPIO_ReadPin(ENC_M6_A_GPIO_Port, ENC_M6_A_Pin);
//		nowB[5] = HAL_GPIO_ReadPin(ENC_M6_B_GPIO_Port, ENC_M6_B_Pin);
//		Enc_count[5] = encoder(5);
//	}

	if (GPIO_Pin == LS1_Pin) {
		ls1_state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
	}
	if (GPIO_Pin == LS2_Pin) {
		ls2_state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);
	}
	if (GPIO_Pin == LS3_Pin) {
		ls3_state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);
	}


}
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
	while (1) {
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

