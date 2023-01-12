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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stdio.h"
#include "Accel_stepper.h" // Stepper Library
#include "cli.h" // Uart Library
#include "MY_FLASH.h" // Flash memory
#include "Inverse_Arm.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*Declare acceleration for each stepper*/
Acceleration_t Stepper1;
Acceleration_t Stepper2;
Acceleration_t Stepper3;
Acceleration_t Stepper4;
Acceleration_t Stepper5;
Acceleration_t Stepper6;
Inv_Arm_t Arm;

/*
.
.
Acceleration_t Stepper#n
*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//Formula compare the value of stepper
#define max(x,y) (x > y) ? x : y;
#define min(x,y) (x > y) ? y : x;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//=================================ARM=========================================//
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;
float theta4 = 0;
float theta5 = 0;
float theta6 = 0;

int set_theta1 = 0;
int set_theta2 = 0;
int set_theta3 = 0;
int set_theta4 = 0;
int set_theta5 = 0;
int set_theta6 = 0;

uint8_t freerun = 1;
int Move_angle1=0;
int Move_angle2=0;
int Move_angle3=0;
int Move_angle4=0;
int Move_angle5=0;
int Move_angle6=0;

uint8_t Lsw1 = 0;
uint8_t Lsw2 = 0;
uint8_t Lsw3 = 0;
uint8_t Lsw4 = 1;
uint8_t Lsw5 = 0;
uint8_t Lsw6 = 0;

uint16_t theta1dot = 0;
uint16_t theta2dot = 0;
uint16_t theta3dot = 0;
uint16_t theta4dot = 0;
uint16_t theta5dot = 0;
uint16_t theta6dot = 0;
uint16_t max_speed = 500;

uint16_t accel1 = 0;
uint16_t accel2 = 0;
uint16_t accel3 = 0;
uint16_t accel4 = 0;
uint16_t accel5 = 0;
uint16_t accel6 = 0;
uint16_t a_max = 2000;

uint32_t step1 = 0;
uint32_t step2 = 0;
uint32_t step3 = 0;
uint32_t step4 = 0;
uint32_t step5 = 0;
uint32_t step6 = 0;

Memory_t memory;
char uart_buffer[100];
char Data[10] = "Hello\n\r";

// Uart_buffer store variable from cmd;
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
	HAL_TIM_Base_Start_IT(&htim14);
	//(&htim7)->Instance->CNT = (0);
	__HAL_TIM_SET_COUNTER(&htim14, 0);
	while(__HAL_TIM_GET_COUNTER(&htim14) < us);
	HAL_TIM_Base_Stop_IT(&htim14);
}
// Compare the value of 6 stepper motor
int max6(int a, int b, int c, int d,int e, int f){
	int n1=max(a,b);
	int n2=max(c,d);
	int n3=max(e,f);
	int n4=max(n1,n2);
    return max(n3,n4);
}
// Limit switch start
void limit_switch (){
//   Lsw1= HAL_GIOP_ReadPin​(GPIOE, GPIO_PIN_0);
     Lsw2= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
     Lsw3= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
     Lsw4= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
     Lsw5= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
     Lsw6= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5);
}
signed int angle2step(float angle, int ratio){
	    return angle*SPR*ratio/360;
//  	return angle*SPR*ratio/360   ;//20=2*gearratio
}
signed int rad2step(float rads, int gear){
//	rads*SPR*10/(2*pi)=rads/ALPHA
	return rads*gear/ALPHA;//20=2*gearratio
}
void HOME(void){
//	static uint8_t rule2= 0;  // static run for only one time
	HAL_GPIO_WritePin(dir_1_GPIO_Port, dir_1_Pin, 0);//CW-
	HAL_GPIO_WritePin(dir_2_GPIO_Port, dir_2_Pin, 1);//CCW+
	HAL_GPIO_WritePin(dir_3_GPIO_Port, dir_3_Pin, 0);//CW-
	HAL_GPIO_WritePin(dir_4_GPIO_Port, dir_4_Pin, 1);//CCW+
	HAL_GPIO_WritePin(dir_5_GPIO_Port, dir_5_Pin, 1);//CW-
    HAL_GPIO_WritePin(dir_6_GPIO_Port, dir_6_Pin, 1);//CCW+
while (Lsw2 ==0 || Lsw3==0 ||Lsw5==0 ){ // if it work in condition 0 when it touch limit switch Lsw=1=> Out of loop
      limit_switch ();// call void limit switch
	if (Lsw2==0){
	HAL_GPIO_TogglePin(step_2_GPIO_Port, step_2_Pin);
	}
	if (Lsw3==0){
	 HAL_GPIO_TogglePin(step_3_GPIO_Port, step_3_Pin);
	}
//    if ( Lsw4==1){
//    	HAL_GPIO_TogglePin(step_4_GPIO_Port, step_4_Pin);// Lsw4==0 when the motor keep in touch with (**Logic AND)
//    }
    if (Lsw5==0){
    	HAL_GPIO_TogglePin(step_5_GPIO_Port, step_5_Pin);
    }
    DelayUs_step(200); // Control pule delay motor
  }
//  while (Lsw1==0){
//	limit_switch();
//	if (Lsw1==0){
//		HAL_GPIO_TogglePin(step_1_GPIO_Port, step_1_Pin);
//	}
//	DelayUs_step(400);  //(Axis-1 move later)
//  }
}
// Limit switch End
//
// To find the max value
void Syn_Stepper(int step1, int step2, int step3, int step4, int step5, int step6){
	int step_max = max6(abs(step1), abs(step2), abs(step3), abs(step4), abs(step5), abs(step6));
	float coef1 = fabs(step1)/step_max;
	float coef2 = fabs(step2)/step_max;
	float coef3 = fabs(step3)/step_max;
	float coef4 = fabs(step4)/step_max;
	float coef5 = fabs(step5)/step_max;
	float coef6 = fabs(step6)/step_max;
	theta1dot = max_speed * coef1;
	theta2dot = max_speed * coef2;
	theta3dot = max_speed * coef3;
	theta4dot = max_speed * coef4;
	theta5dot = max_speed * coef5;
	theta6dot = max_speed * coef6;
	accel1 = a_max * coef1;
	accel2 = a_max * coef2;
	accel3 = a_max * coef3;
	accel4 = a_max * coef4;
	accel5 = a_max * coef5;
	accel6 = a_max * coef6;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  stepper_motor_set_param(MOTOR_X, 100);
	  Accel_Stepper_SetPin(&Stepper1, step_1_GPIO_Port, step_1_Pin, dir_1_GPIO_Port, dir_1_Pin);
	  Accel_Stepper_SetPin(&Stepper2, step_2_GPIO_Port, step_2_Pin, dir_2_GPIO_Port, dir_2_Pin);
	  Accel_Stepper_SetPin(&Stepper3, step_3_GPIO_Port, step_3_Pin, dir_3_GPIO_Port, dir_3_Pin);
	  Accel_Stepper_SetPin(&Stepper4, step_4_GPIO_Port, step_4_Pin, dir_4_GPIO_Port, dir_4_Pin);
	  Accel_Stepper_SetPin(&Stepper5, step_5_GPIO_Port, step_5_Pin, dir_5_GPIO_Port, dir_5_Pin);
	  Accel_Stepper_SetPin(&Stepper6, step_6_GPIO_Port, step_6_Pin, dir_6_GPIO_Port, dir_6_Pin);
	  Accel_Stepper_SetTimer(&Stepper1, &htim1);
	  Accel_Stepper_SetTimer(&Stepper2, &htim2);
	  Accel_Stepper_SetTimer(&Stepper3, &htim3);
      Accel_Stepper_SetTimer(&Stepper4, &htim4);
	  Accel_Stepper_SetTimer(&Stepper5, &htim5);
	  Accel_Stepper_SetTimer(&Stepper6, &htim6);
	  HAL_Delay(2000);
// Testing motor
//	  Accel_Stepper_Move(&Stepper1, angle2step(60,5), 1000, 1000, 200);
//	  Accel_Stepper_Move(&Stepper2, angle2step(60,20), 1000, 1000, 200);
//	  Accel_Stepper_Move(&Stepper3, angle2step(60,20), 1000, 1000, 200);
//	  Accel_Stepper_Move(&Stepper4, angle2step(60,14), 1000, 1000, 200);
//	  Accel_Stepper_Move(&Stepper5, angle2step(30,14), 1000, 1000, 200);
//	  Accel_Stepper_Move(&Stepper6, angle2step(60,14), 1000, 1000, 200);
// Testing Power Input
//	  HAL_GPIO_WritePin(dir_2_GPIO_Port, dir_2_Pin, 0);
//	  HAL_GPIO_WritePin(step_2_GPIO_Port, step_2_Pin, 0);

HOME(); // Call the function Home to use
//	Initial angle
	  	theta1 = 90;  // Set_Point as Home
	  	theta2 = 90;
	  	theta3 = 90;
//	  	theta4 = 0;
	  	theta5 = 90;
	  	theta6 = 0;
	  	Accel_Stepper_Move(&Stepper1, angle2step(0,5) , 1000, 1000, 200);
	  	Accel_Stepper_Move(&Stepper2, angle2step(-90,20) , 1000, 1000, 200);
	  	Accel_Stepper_Move(&Stepper3, angle2step(90,20) , 1000, 1000, 200);
//	  	Accel_Stepper_Move(&Stepper4, angle2step(0,14) , 1000, 1000, 200);
		Accel_Stepper_Move(&Stepper5, angle2step(-90,14) , 1000, 1000, 200);
//		angle2step(angle move back after get touch with limit switch to Home, ratio);


CLI_Init(&huart1);
MY_FLASH_SetSectorAddrs(11, 0x080E5000); // address flash memory
//MY_FLASH_SetSectorAddrs(sector, address);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_UART_Transmit(&huart1,(uint8_t*) &Data, sizeof(Data), 100);
//	  HAL_Delay(1000);

	  static uint8_t count_arm = 0;

	 if(Stepper1.run_status != 1 && Stepper2.run_status != 1 && Stepper3.run_status != 1 && Stepper5.run_status != 1 && Stepper6.run_status != 1){
	 	 	  if(freerun == 1){
	 			  switch(count_arm){

//	 			Cinematic Mode
	 			  case 0:
	 				  Arm.theta1 = 20;
	 				  Arm.theta2 = 80;
	 				  Arm.theta3 = 60;
//	 			      Arm.theta4 = 0;
	 				  Arm.theta5 = 130;
	 				  Arm.theta6 = 90;
	 				  break;
	 			  case 1:
	 				  Arm.theta1 = 90;
	 				  Arm.theta2 = 60;
	 				  Arm.theta3 = 90;
//	 			      Arm.theta4 = 0;
	 				  Arm.theta5 = 120;
	 				  Arm.theta6 = 0;
	 				  break;

	 			  case 2:
	 				  Arm.theta1 = 120;
	 				  Arm.theta2 = 100;
	 				  Arm.theta3 = 60;
//	 				  Arm.theta4 = 0;
	 				  Arm.theta5 = 120;
	 				  Arm.theta6 = -90;
	 				 break;
	 			  case 3:
	 				  Arm.theta1 = 160;
	 				  Arm.theta2 = 90;
	 				  Arm.theta3 = 60;
//	 				  Arm.theta4 = 0;
	 				  Arm.theta5 =120;
	 				  Arm.theta6 = 0;
	 				 break;
	 			  }
	 			  Move_angle1 = angle2step(Arm.theta1 - theta1, 5);
	 			  Move_angle2 = angle2step(Arm.theta2 - theta2, 20);
	 			  Move_angle3 = angle2step(Arm.theta3 - theta3, 20);
//	 		      Move_angle4 = angle2step(Arm.theta4 - theta4, 14);
	 			  Move_angle5 = angle2step(Arm.theta5 - theta5, 14);
	 			  Move_angle6 = angle2step(Arm.theta6 - theta6, 14);

	 			  Syn_Stepper(Move_angle1, Move_angle2, Move_angle3, Move_angle4, Move_angle5, Move_angle6);

	 			  Accel_Stepper_Move(&Stepper1, Move_angle1, accel1, accel1, theta1dot);
	 			  Accel_Stepper_Move(&Stepper2, Move_angle2, accel2, accel2, theta2dot);
	 			  Accel_Stepper_Move(&Stepper3, Move_angle3, accel3, accel3, theta3dot);
//	 		      Accel_Stepper_Move(&Stepper4, Move_angle4, accel4, accel4, theta4dot);
	 			  Accel_Stepper_Move(&Stepper5, Move_angle5, accel5, accel5, theta5dot);
	 			  Accel_Stepper_Move(&Stepper6, Move_angle6, accel6, accel6, theta6dot);

	 			  theta1 = Arm.theta1; // Set theta to update
	 			  theta2 = Arm.theta2;
	 			  theta3 = Arm.theta3;
//	 		      theta4 = Arm.theta4;
	 			  theta5 = Arm.theta5;
	 			  theta6 = Arm.theta6;
//	 			  Drawing Mode
//	 			 case 0:
//	 			   Arm.theta1= 90;
//	 			   Arm.theta2= 90;
//	 			   Arm.theta3= 60;
//	 			   Arm.theta5= 110;
//	 			   Arm.theta6= 0;
//	 			break;
//	 			 case 1:
//	 			   Arm.theta1= 80;
//	 			   Arm.theta2= 90;
//	 			   Arm.theta3= 100;
//	 			   Arm.theta5= 110;
//	 			   Arm.theta6= 0;
//	 			 break;
////	 			 case 2:
////	 		  Arm.theta1= 90;
////	 		  Arm.theta2= 90;
////	 		  Arm.theta3= 90;
////	 		  Arm.theta5= 110;
////	 		  Arm.theta6= 0;
////	 		break;
////	 		case 3:
////	 		Arm.theta1= 80;
////	 		Arm.theta2= 90;
////	 		Arm.theta3= 100;
////	 	    Arm.theta5=110;
////	 	    Arm.theta6= 0;
////	 	    break;
//	 			  }
//	 	     Move_angle1 = angle2step(Arm.theta1 - theta1, 5);
//	 	     Move_angle2 = angle2step(Arm.theta2 - theta2, 20);
//           Move_angle3 = angle2step(Arm.theta3 - theta3, 20);
//	 	     Move_angle5 = angle2step(Arm.theta5 - theta5, 14);
//	 	     Move_angle6 = angle2step(Arm.theta6 - theta6, 14);
//
//          Syn_Stepper(Move_angle1, Move_angle2, Move_angle3, Move_angle4, Move_angle5, Move_angle6);

//	        Accel_Stepper_Move(&Stepper1, Move_angle1, accel1, accel1, theta1dot);
//	 	    Accel_Stepper_Move(&Stepper2, Move_angle2, accel2, accel2, theta2dot);
//	        Accel_Stepper_Move(&Stepper3, Move_angle3, accel3, accel3, theta3dot);
//	        Accel_Stepper_Move(&Stepper5, Move_angle5, accel5, accel5, theta5dot);
//          Accel_Stepper_Move(&Stepper6, Move_angle6, accel6, accel6, theta6dot);

//	 		 theta1 = Arm.theta1;
//	 	     theta2 = Arm.theta2;
//	 		 theta2 = Arm.theta3;
//	 	     theta5 = Arm.theta5;
//	 		 theta6 = Arm.theta6;
	 		  }
	 		 count_arm = count_arm + 1;
	 		 if(count_arm > 3) count_arm = 0;
	  }
    // UART Start
    CLI_Process(); // Call the CLI Library to use
	char* msg=strtok(uart_buffer," ");//Get msg and store by uart_buffer
	  if(strcmp(msg, "set_home")==0){ //After get msg string compare start comparing the value (msg and set_home); (*Logic AND)
		                              //The both of two value should the same by  using string compare
		HOME();
		theta1 = 90;
		theta2 = 90;
		theta3 = 90;
		theta4 = 0;
		theta5 = 90;
		theta6 = 0;
		CLI_Printf("Done!");
	 	memset(uart_buffer, 0, 100);
	  }
	   if (strcmp(msg, "set_amax")==0){
	      char* cmd=strtok(uart_buffer, " ");
	   if (cmd !=NULL){
	    	  char* arg = strtok(NULL, " "); // read the string after x (read char from cmd)
	    	   uint32_t val = atoi(arg);    // Read ascii computer Language
	    	   memcpy(&memory.a_max, &val, sizeof(uint32_t));
	    	   memset(uart_buffer, 0, 100);
//	    	   memset (void *, int, size_t); //Reset cmd that store by FLASH MEMORIE
	    	                                //In this use to store the value of a_max; v_max and theta read from cmd
	   }
	  }
	   if (strcmp(msg,"set_vmax")==0){
		   char* cmd=strtok(uart_buffer," ");
	     if (cmd!=NULL){
		   char* arg = strtok(NULL, " "); // Read cmd form UART to clear value one by one  // Read cmd after x
		   uint32_t val = atoi(arg);
		   memcpy(&memory.v_max, &val, sizeof(uint32_t));
		   memset(uart_buffer, 0, 100);
	     }
	   }
	   if (strcmp(msg,"theta")==0){
		   char* cmd=strtok(uart_buffer," ");
	   if (cmd !=NULL){
		   Inv_Arm_t Arm;
		   char* arg1=strtok(NULL,"");
		   char* arg2=strtok(NULL,"");
		   char* arg3=strtok(NULL,"");
		   char* arg4=strtok(NULL,"");
		   char* arg5=strtok(NULL,"");
		   char* arg6=strtok(NULL,"");
		   Arm.theta1 = atof(arg1);
		   Arm.theta2 = atof(arg2);
		   Arm.theta3 = atof(arg3);
		   Arm.theta4 = atof(arg4);
		   Arm.theta5 = atof(arg5);
		   Arm.theta6 = atof(arg6);
		   memset(uart_buffer, 0, 100);
		   step1 = angle2step(Arm.theta1, 5); //angle2step(angle,ratio of gear box motor)
		   step2 = angle2step(Arm.theta2,20);
		   step3 = angle2step(Arm.theta3,20);
		   step4 = angle2step(Arm.theta4,14);
		   step5 = angle2step(Arm.theta5,14);
		   step6 = angle2step(Arm.theta6,14);
	     }
    }
// UART END
//--------------------------------------------------------------
//	           step  =angle2step(angle, ratio of gear box motor);
//	           step1 = angle2step(180,10);
//	   		   step2 = angle2step(45-theta2,20);
//	   		   step3 = angle2step(60-theta3,20);
////	   	   step4 = angle2step(360,14);
//	   		   step5 = angle2step(170-theta5,14);
//	   		   step6 = angle2step(180,14);
//	       Syn_Stepper(step1, step2,step3,step4,step5,step6);
//  if(Stepper1.run_status == 0){
//		    Accel_Stepper_Move(&Stepper1, step1 , accel1, accel1, theta1dot);// acceleration=deceleration
////	    Accel_Stepper_Move(&Stepper1, step1 , accel1, decel1, speed);
//	  }
//	if(Stepper2.run_status == 0){
//		    Accel_Stepper_Move(&Stepper2, step2 , accel2, accel2, theta2dot);
////		    theta2=45; // angle move after motor stop
//	}
//	if(Stepper3.run_status == 0){
//			Accel_Stepper_Move(&Stepper3, step3 , accel3, accel3, theta3dot);
////			theta3=60;
//	}
//	if(Stepper4.run_status == 0){
//			Accel_Stepper_Move(&Stepper4, step4 , accel4, accel4, theta4dot);
//		}
//	if(Stepper5.run_status == 0){
//			Accel_Stepper_Move(&Stepper5, step5 , accel5, accel5, theta5dot);
////           theta5=170;
//		}
//	if(Stepper6.run_status == 0){
//			Accel_Stepper_Move(&Stepper6, step6 , accel6, accel6, theta6dot);
//		}
	HAL_Delay(1000);
//	limit_switch (); // Call the limit switch to test
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	  if(htim->Instance == TIM1){//stepper1
		      Accel_Stepper_TIMIT_Handler(&Stepper1);
	  }
	  if(htim->Instance == TIM2){//stepper2
		      Accel_Stepper_TIMIT_Handler(&Stepper2);
	  }
	  if(htim->Instance == TIM3){//stepper3
	 		  Accel_Stepper_TIMIT_Handler(&Stepper3);
	 	  }
	  if(htim->Instance == TIM4){//stepper4
	 		  Accel_Stepper_TIMIT_Handler(&Stepper4);
	 	  }
	  if(htim->Instance == TIM5){//stepper5
	 		  Accel_Stepper_TIMIT_Handler(&Stepper5);
	 	  }
	  if(htim->Instance == TIM6){//stepper6
	 		  Accel_Stepper_TIMIT_Handler(&Stepper6);
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

