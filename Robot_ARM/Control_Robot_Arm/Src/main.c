/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Control Robot Arm 6DOF
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 HOKKEY.
  * All rights reserved.
  *
  *
  *
  *
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
#include "stdbool.h"
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
Inv_Arm_t theta;
//Inv_Arm_t Inverse;
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
#define max(a,b) (a > b) ? a : b
#define min(a,b) (a > b) ? b : a
#define pi  3.1416
bool state=0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//=================================ARM=========================================//
float theta1=0;
float theta2=0;
float theta3=0;
float theta4=0;
float theta5=0;
float theta6=0;

uint8_t freerun = 1;
int Move_angle1=0;
int Move_angle2=0;
int Move_angle3=0;
int Move_angle4=0;
int Move_angle5=0;
int Move_angle6=0;

uint8_t Lsw1 = 1;
uint8_t Lsw2 = 0;
uint8_t Lsw3 = 0;
uint8_t Lsw4 = 1;
uint8_t Lsw5 = 0;

uint16_t theta1dot = 0;
uint16_t theta2dot = 0;
uint16_t theta3dot = 0;
uint16_t theta4dot = 0;
uint16_t theta5dot = 0;
uint16_t theta6dot = 0;
uint16_t max_speed = 200;

uint16_t accel1 = 0;
uint16_t accel2 = 0;
uint16_t accel3 = 0;
uint16_t accel4 = 0;
uint16_t accel5 = 0;
uint16_t accel6 = 0;
uint16_t a_max = 500;

int step1 = 0;
int step2 = 0;
int step3 = 0;
int step4 = 0;
int step5 = 0;
int step6 = 0;

Memory_t memory;
char uart_buffer[100];
//----------------------------
//Inverse Kinematics (3 Joint)
//DH Parameter Variable( a,d,alpha,theta)
//float Alpha1=pi/2; //* alpha= angle(Z(i-1),Z(i)) along to X(i-1)
//float Alpha2=0;
//float Alpha3=pi/2;
int A1=0;  //* a= distan(Z(i-1),Z(i)) belong to X(i-1)
int A2=390;
int A3=70;
int A4=0;
int A5=0;
int A6=0;

int D1=220; //* d=distan(X(i-1),X(i)) belong to Z(i)
int D2=0;
int D3=0;
int D4=363;
int D5=0;
int D6=92.7;

float alpha1 = pi/2;
float alpha2 = 0;
float alpha3 = pi/2;
float alpha4 = -pi/2;
float alpha5 = pi/2;
float alpha6 = 0;

float Eq1;
float Eq2;
float Eq3;
float Eq4;

float R13;
float R23;
float R31;
float R32;
float R33;

float theta11;
float theta22;
float theta33;
float theta44;
float theta55;
float theta66;

typedef struct {
	float x;
	float y;
	float z;
	float Rx;
	float Ry;
	float Rz;
}ARM_POS;
ARM_POS setpos;
ARM_POS savepos;

typedef struct {
	float THETA1;
	float THETA2;
	float THETA3;
	float THETA4;
	float THETA5;
	float THETA6;
}THETA;
THETA EQ;

//-----------------------------

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
	 Lsw1= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
     Lsw2= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
     Lsw3= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
     Lsw4= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
     Lsw5= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
}
signed int angle2step(float angle, int ratio){
	    return angle*SPR*ratio/360;
}
signed int rad2step(float rads, int gear){
      //	rads*SPR*10/(2*pi)=rads/ALPHA
	return rads*gear/ALPHA;
}
void Arm_Limit(float *angle1, float *angle2, float *angle3, float *angle4,float *angle5, float *angle6){
   float *tmp1;
   float *tmp2;
   float *tmp3;
   float *tmp4;
   float *tmp5;
   float *tmp6;

    tmp1 = (float *)angle1;
        if(*angle1<0) {
          (*(float *) tmp1) = 0;
//          step1=0;
    }
    else if(*angle1>180) {
         (*(float *) tmp1) = 180;
//         step1=0;
    }
    tmp2 = (float *)angle2;
        if(*angle2<0) {
          (*(float *) tmp2) = 0;
//          step2=0;
       }
       else if(*angle2>180) {
            (*(float *) tmp2) = 180;
//          step2=0;
        }
    tmp3 = (float *)angle3;
        if(*angle3<0) {
          (*(float *) tmp3) = 0;
//          step3=0;
        }
        else if(*angle3>180) {
             (*(float *) tmp3) = 180;
//          step3=0;
        }
    tmp4 = (float *)angle4;
        if(*angle4<-90) {
          (*(float *) tmp4) = -90;
//          step4=0;
        }
        else if(*angle4>90) {
             (*(float *) tmp4) = 90;
//          step4=0;
        }
    tmp5 = (float *)angle5;
        if(*angle5<-90) {
          (*(float *) tmp5) =-90;
         // step5=0;
        }
        else if(*angle5>90) {
             (*(float *) tmp5) =90;
//          step5=0;
        }
    tmp6 = (float *)angle6;
        if(*angle6<0) {
          (*(float *) tmp6) = 0;
//         step6=0;
        }
         else if(*angle6>360) {
              (*(float *) tmp6) = 360;
//         step6=0;
     }
}

void HOME(void){
	static uint8_t L2sw1= 0;
	static uint8_t L2sw4= 0;  // static run for only one time

//	HAL_GPIO_WritePin(dir_1_GPIO_Port, dir_1_Pin, 0);//CW-
	HAL_GPIO_WritePin(dir_2_GPIO_Port, dir_2_Pin, 1);//CCW+
	HAL_GPIO_WritePin(dir_3_GPIO_Port, dir_3_Pin, 0);//CW-
//	HAL_GPIO_WritePin(dir_4_GPIO_Port, dir_4_Pin, 1);
	HAL_GPIO_WritePin(dir_5_GPIO_Port, dir_5_Pin, 1);//CCW+
	HAL_Delay(100);

do{ // if it work in condition 0 when it touch limit switch Lsw=1=> Out of loop

	limit_switch ();// call void limit switch
    if (Lsw1==1 && L2sw1==0){
    	if (EQ.THETA1 <40){
    		HAL_GPIO_WritePin(dir_1_GPIO_Port, dir_1_Pin, 0);//CCW+

    	}else HAL_GPIO_WritePin(dir_1_GPIO_Port, dir_1_Pin, 1);//CCW+

    	HAL_GPIO_TogglePin(step_1_GPIO_Port, step_1_Pin);
       	L2sw1 = 1;
    }
    else{
	    L2sw1 = 0;

    }
	if (Lsw2==0){
	HAL_GPIO_TogglePin(step_2_GPIO_Port, step_2_Pin);
	}
	if (Lsw3==0){
	 HAL_GPIO_TogglePin(step_3_GPIO_Port, step_3_Pin);
	}
    if ( Lsw4==1 && L2sw4==0){
    	if (EQ.THETA4 >0){
    		HAL_GPIO_WritePin(dir_4_GPIO_Port, dir_4_Pin, 0);//CCW+

    	}else HAL_GPIO_WritePin(dir_4_GPIO_Port, dir_4_Pin, 1);//CCW+

    	HAL_GPIO_TogglePin(step_4_GPIO_Port, step_4_Pin);// Lsw4==0 when the motor keep in touch with (**Logic AND)
       	L2sw4 = 1;
    }
    else{
	    L2sw4 = 0;
    }
    if (Lsw5==0){
    	HAL_GPIO_TogglePin(step_5_GPIO_Port, step_5_Pin);
    }
    DelayUs_step(200); // Control pule delay motor
  }while ((Lsw1==1 && L2sw1==0)||Lsw2==0|| Lsw3==0||(Lsw4==1 && L2sw4==0)|| Lsw5==0);
}
// Limit switch End
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
// DH parameter (thelta,d,a,alpha);
// Inverse Kinematic
//void Inverse_3Joint(float x, float y, float z){
//	theta11 = atan2(y,x);
//	Eq1 =(pow(x,2)+pow(y,2)+pow((z-D1),2)-pow(A2,2)-pow(A3,2))/(2*A2*A3);
//    theta33 = atan2(-sqrt(fabs(1-pow(Eq1,2))),Eq1);
//	Eq2 = atan2(A3*sin(theta33),A2+(A3*cos(theta33)));
//	Eq3 = atan2(z-D1,sqrt(fabs(pow(x, 2)+pow(y,2))));
//	theta22 = Eq3-Eq2;
//	EQ.THETA1 = theta11*180./pi;
//	EQ.THETA2 = theta22*180./pi;
//	EQ.THETA3 = 180+theta33*180./pi;
//}
void Inverse_6Joint(float x, float y, float z,float Rx, float Ry, float Rz){

//	float Rz = 72.586*pi/180;
//	float Rx = -103.007*pi/180;
//	float Ry = 33.997*pi/180;

//	float Rz = 2*pi;
//	float Rx = 0;
//	float Ry = pi/2;

	float rr13 = cos(Ry)*sin(Rz);
	float rr23 = sin(Ry)*sin(Rz);
	float rr33 = cos(Rz);

	float xw = x - D6*rr13;
	float yw = y - D6*rr23;
	float zw = z - D6*rr33;

	theta11 = atan2(yw,xw);
	Eq1 = sqrt(pow(A3,2)+pow(D4,2));
	float Alpha = atan2(A3,D4);
	Eq2 = (pow(xw,2)+pow(yw,2)+pow((zw-D1),2)-pow(A2,2)-pow(Eq1,2))/(2*A2*Eq1);
	theta33 = atan2(-sqrt(fabs(1-pow(Eq2,2))),Eq2)+pi/2-Alpha;
	Eq3 = atan2(-Eq1*cos(Alpha + theta33),A2+(Eq1*sin(Alpha + theta33)));
	Eq4 = atan2(zw-D1,xw/cos(theta11));
	theta22 = Eq4-Eq3;

	R33 = (cos(Rz)*(sin(alpha1)*pow(sin(alpha2),2)*sin(alpha3)*sin(theta22)*sin(theta33) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(cos(theta22),2)*pow(cos(theta33),2) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(cos(theta22),2)*pow(sin(theta33),2) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(cos(theta33),2)*pow(sin(theta22),2) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(sin(theta22),2)*pow(sin(theta33),2) - cos(alpha2)*sin(alpha1)*sin(alpha3)*cos(theta22)*cos(theta33) - cos(alpha1)*sin(alpha2)*sin(alpha3)*pow(cos(theta22),2)*cos(theta33) - cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta22)*pow(cos(theta33),2) - cos(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta33)*pow(sin(theta22),2) - cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta22)*pow(sin(theta33),2) + pow(cos(alpha2),2)*sin(alpha1)*sin(alpha3)*sin(theta22)*sin(theta33)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) + (cos(Ry)*sin(Rz)*(cos(alpha2)*cos(alpha3)*sin(alpha1)*pow(cos(theta22),2)*pow(cos(theta33),2)*sin(theta11) + pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*cos(theta11)*pow(cos(theta33),2)*sin(theta22) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha2)*cos(alpha3)*sin(alpha1)*pow(cos(theta22),2)*sin(theta11)*pow(sin(theta33),2) + cos(alpha2)*cos(alpha3)*sin(alpha1)*pow(cos(theta33),2)*sin(theta11)*pow(sin(theta22),2) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*cos(theta11)*pow(cos(theta33),2)*sin(theta22) + pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*cos(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha2)*cos(alpha3)*sin(alpha1)*sin(theta11)*pow(sin(theta22),2)*pow(sin(theta33),2) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*cos(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha1)*cos(alpha2)*sin(alpha3)*cos(theta22)*cos(theta33)*sin(theta11) + cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta22)*pow(cos(theta33),2)*sin(theta11) + pow(cos(alpha1),2)*cos(alpha2)*sin(alpha3)*cos(theta11)*cos(theta33)*sin(theta22) + cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta22)*sin(theta11)*pow(sin(theta33),2) + cos(alpha2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta11)*cos(theta33)*sin(theta22) - cos(alpha1)*pow(cos(alpha2),2)*sin(alpha3)*sin(theta11)*sin(theta22)*sin(theta33) - sin(alpha1)*sin(alpha2)*sin(alpha3)*pow(cos(theta22),2)*cos(theta33)*sin(theta11) - cos(alpha1)*pow(sin(alpha2),2)*sin(alpha3)*sin(theta11)*sin(theta22)*sin(theta33) - sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta33)*sin(theta11)*pow(sin(theta22),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) + (sin(Ry)*sin(Rz)*(pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*pow(cos(theta33),2)*sin(theta11)*sin(theta22) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(cos(theta22),2)*pow(sin(theta33),2) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(cos(theta33),2)*pow(sin(theta22),2) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(sin(theta22),2)*pow(sin(theta33),2) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*pow(cos(theta33),2)*sin(theta11)*sin(theta22) + pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*sin(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*sin(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) - cos(alpha1)*cos(alpha2)*sin(alpha3)*cos(theta11)*cos(theta22)*cos(theta33) - cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta11)*cos(theta22)*pow(cos(theta33),2) - cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta11)*cos(theta22)*pow(sin(theta33),2) + cos(alpha1)*pow(cos(alpha2),2)*sin(alpha3)*cos(theta11)*sin(theta22)*sin(theta33) + sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta11)*pow(cos(theta22),2)*cos(theta33) + pow(cos(alpha1),2)*cos(alpha2)*sin(alpha3)*cos(theta33)*sin(theta11)*sin(theta22) + cos(alpha1)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta11)*sin(theta22)*sin(theta33) + cos(alpha2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta33)*sin(theta11)*sin(theta22) + sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta11)*cos(theta33)*pow(sin(theta22),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2));
	R23 = (cos(Rz)*(cos(alpha1)*cos(alpha2)*sin(alpha3)*pow(cos(theta22),2)*pow(cos(theta33),2) + cos(alpha1)*cos(alpha2)*sin(alpha3)*pow(cos(theta22),2)*pow(sin(theta33),2) + cos(alpha1)*cos(alpha2)*sin(alpha3)*pow(cos(theta33),2)*pow(sin(theta22),2) + cos(alpha1)*cos(alpha2)*sin(alpha3)*pow(sin(theta22),2)*pow(sin(theta33),2) + cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta22)*cos(theta33) + cos(alpha1)*cos(alpha3)*sin(alpha2)*pow(cos(theta22),2)*cos(theta33) + cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta33)*pow(sin(theta22),2) - sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta22)*pow(cos(theta33),2) - pow(cos(alpha2),2)*cos(alpha3)*sin(alpha1)*sin(theta22)*sin(theta33) - cos(alpha3)*sin(alpha1)*pow(sin(alpha2),2)*sin(theta22)*sin(theta33) - sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta22)*pow(sin(theta33),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) - (sin(Ry)*sin(Rz)*(cos(alpha2)*sin(alpha1)*sin(alpha3)*cos(theta11)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*cos(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + cos(alpha2)*sin(alpha1)*sin(alpha3)*cos(theta11)*pow(cos(theta22),2)*pow(sin(theta33),2) + cos(alpha2)*sin(alpha1)*sin(alpha3)*cos(theta11)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*cos(alpha3)*pow(sin(alpha2),2)*cos(theta22)*sin(theta11)*sin(theta33) + pow(cos(alpha2),2)*cos(alpha3)*pow(sin(alpha1),2)*cos(theta22)*sin(theta11)*sin(theta33) + cos(alpha2)*sin(alpha1)*sin(alpha3)*cos(theta11)*pow(sin(theta22),2)*pow(sin(theta33),2) + cos(alpha3)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*cos(theta22)*sin(theta11)*sin(theta33) - pow(cos(alpha1),2)*sin(alpha2)*sin(alpha3)*pow(cos(theta33),2)*sin(theta11)*sin(theta22) - cos(alpha1)*cos(alpha2)*cos(alpha3)*cos(theta11)*cos(theta22)*cos(theta33) - pow(cos(alpha1),2)*sin(alpha2)*sin(alpha3)*sin(theta11)*sin(theta22)*pow(sin(theta33),2) - pow(sin(alpha1),2)*sin(alpha2)*sin(alpha3)*pow(cos(theta33),2)*sin(theta11)*sin(theta22) - pow(sin(alpha1),2)*sin(alpha2)*sin(alpha3)*sin(theta11)*sin(theta22)*pow(sin(theta33),2) + cos(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta11)*cos(theta22)*pow(cos(theta33),2) + cos(alpha1)*pow(cos(alpha2),2)*cos(alpha3)*cos(theta11)*sin(theta22)*sin(theta33) + cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta11)*pow(cos(theta22),2)*cos(theta33) + pow(cos(alpha1),2)*cos(alpha2)*cos(alpha3)*cos(theta33)*sin(theta11)*sin(theta22) + cos(alpha1)*cos(alpha3)*pow(sin(alpha2),2)*cos(theta11)*sin(theta22)*sin(theta33) + cos(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta11)*cos(theta22)*pow(sin(theta33),2) + cos(alpha2)*cos(alpha3)*pow(sin(alpha1),2)*cos(theta33)*sin(theta11)*sin(theta22) + cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta11)*cos(theta33)*pow(sin(theta22),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) + (cos(Ry)*sin(Rz)*(cos(alpha2)*sin(alpha1)*sin(alpha3)*pow(cos(theta22),2)*pow(cos(theta33),2)*sin(theta11) - pow(cos(alpha1),2)*cos(alpha3)*pow(sin(alpha2),2)*cos(theta11)*cos(theta22)*sin(theta33) - pow(cos(alpha2),2)*cos(alpha3)*pow(sin(alpha1),2)*cos(theta11)*cos(theta22)*sin(theta33) - pow(cos(alpha1),2)*pow(cos(alpha2),2)*cos(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) - cos(alpha3)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*cos(theta11)*cos(theta22)*sin(theta33) + pow(cos(alpha1),2)*sin(alpha2)*sin(alpha3)*cos(theta11)*pow(cos(theta33),2)*sin(theta22) + cos(alpha2)*sin(alpha1)*sin(alpha3)*pow(cos(theta22),2)*sin(theta11)*pow(sin(theta33),2) + cos(alpha2)*sin(alpha1)*sin(alpha3)*pow(cos(theta33),2)*sin(theta11)*pow(sin(theta22),2) + pow(cos(alpha1),2)*sin(alpha2)*sin(alpha3)*cos(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(sin(alpha1),2)*sin(alpha2)*sin(alpha3)*cos(theta11)*pow(cos(theta33),2)*sin(theta22) + cos(alpha2)*sin(alpha1)*sin(alpha3)*sin(theta11)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*sin(alpha2)*sin(alpha3)*cos(theta11)*sin(theta22)*pow(sin(theta33),2) - cos(alpha1)*cos(alpha2)*cos(alpha3)*cos(theta22)*cos(theta33)*sin(theta11) - pow(cos(alpha1),2)*cos(alpha2)*cos(alpha3)*cos(theta11)*cos(theta33)*sin(theta22) - cos(alpha2)*cos(alpha3)*pow(sin(alpha1),2)*cos(theta11)*cos(theta33)*sin(theta22) + cos(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta22)*pow(cos(theta33),2)*sin(theta11) + cos(alpha1)*pow(cos(alpha2),2)*cos(alpha3)*sin(theta11)*sin(theta22)*sin(theta33) + cos(alpha3)*sin(alpha1)*sin(alpha2)*pow(cos(theta22),2)*cos(theta33)*sin(theta11) + cos(alpha1)*cos(alpha3)*pow(sin(alpha2),2)*sin(theta11)*sin(theta22)*sin(theta33) + cos(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta22)*sin(theta11)*pow(sin(theta33),2) + cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta33)*sin(theta11)*pow(sin(theta22),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2));
	R13 = (cos(Rz)*(sin(alpha1)*cos(theta33)*pow(cos(alpha2),2)*sin(theta22) + sin(alpha1)*sin(theta33)*cos(alpha2)*cos(theta22) + sin(alpha1)*cos(theta33)*pow(sin(alpha2),2)*sin(theta22) + cos(alpha1)*sin(theta33)*sin(alpha2)*pow(cos(theta22),2) + cos(alpha1)*sin(theta33)*sin(alpha2)*pow(sin(theta22),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) + (cos(Ry)*sin(Rz)*(sin(alpha1)*sin(alpha2)*pow(cos(theta22),2)*sin(theta11)*sin(theta33) - cos(alpha2)*pow(sin(alpha1),2)*cos(theta11)*sin(theta22)*sin(theta33) - cos(alpha1)*pow(sin(alpha2),2)*cos(theta33)*sin(theta11)*sin(theta22) + sin(alpha1)*sin(alpha2)*sin(theta11)*pow(sin(theta22),2)*sin(theta33) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*cos(theta11)*cos(theta22)*cos(theta33) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*cos(theta11)*cos(theta22)*cos(theta33) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*cos(theta11)*cos(theta22)*cos(theta33) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*cos(theta11)*cos(theta22)*cos(theta33) - cos(alpha1)*cos(alpha2)*cos(theta22)*sin(theta11)*sin(theta33) - cos(alpha1)*pow(cos(alpha2),2)*cos(theta33)*sin(theta11)*sin(theta22) - pow(cos(alpha1),2)*cos(alpha2)*cos(theta11)*sin(theta22)*sin(theta33)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) + (sin(Ry)*sin(Rz)*(pow(cos(alpha1),2)*pow(cos(alpha2),2)*cos(theta22)*cos(theta33)*sin(theta11) - pow(cos(alpha1),2)*cos(alpha2)*sin(theta11)*sin(theta22)*sin(theta33) - cos(alpha2)*pow(sin(alpha1),2)*sin(theta11)*sin(theta22)*sin(theta33) - sin(alpha1)*sin(alpha2)*cos(theta11)*pow(sin(theta22),2)*sin(theta33) - sin(alpha1)*sin(alpha2)*cos(theta11)*pow(cos(theta22),2)*sin(theta33) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*cos(theta22)*cos(theta33)*sin(theta11) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*cos(theta22)*cos(theta33)*sin(theta11) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*cos(theta22)*cos(theta33)*sin(theta11) + cos(alpha1)*cos(alpha2)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha1)*pow(cos(alpha2),2)*cos(theta11)*cos(theta33)*sin(theta22) + cos(alpha1)*pow(sin(alpha2),2)*cos(theta11)*cos(theta33)*sin(theta22)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2));
	R32 = ((cos(Ry)*cos(Rx) - cos(Rz)*sin(Ry)*sin(Rx))*(pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*pow(cos(theta33),2)*sin(theta11)*sin(theta22) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(cos(theta22),2)*pow(sin(theta33),2) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(cos(theta33),2)*pow(sin(theta22),2) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(sin(theta22),2)*pow(sin(theta33),2) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*pow(cos(theta33),2)*sin(theta11)*sin(theta22) + pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*sin(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*sin(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) - cos(alpha1)*cos(alpha2)*sin(alpha3)*cos(theta11)*cos(theta22)*cos(theta33) - cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta11)*cos(theta22)*pow(cos(theta33),2) - cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta11)*cos(theta22)*pow(sin(theta33),2) + cos(alpha1)*pow(cos(alpha2),2)*sin(alpha3)*cos(theta11)*sin(theta22)*sin(theta33) + sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta11)*pow(cos(theta22),2)*cos(theta33) + pow(cos(alpha1),2)*cos(alpha2)*sin(alpha3)*cos(theta33)*sin(theta11)*sin(theta22) + cos(alpha1)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta11)*sin(theta22)*sin(theta33) + cos(alpha2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta33)*sin(theta11)*sin(theta22) + sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta11)*cos(theta33)*pow(sin(theta22),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) - ((cos(Rx)*sin(Ry) + cos(Ry)*cos(Rz)*sin(Rx))*(cos(alpha2)*cos(alpha3)*sin(alpha1)*pow(cos(theta22),2)*pow(cos(theta33),2)*sin(theta11) + pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*cos(theta11)*pow(cos(theta33),2)*sin(theta22) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha2)*cos(alpha3)*sin(alpha1)*pow(cos(theta22),2)*sin(theta11)*pow(sin(theta33),2) + cos(alpha2)*cos(alpha3)*sin(alpha1)*pow(cos(theta33),2)*sin(theta11)*pow(sin(theta22),2) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*cos(theta11)*pow(cos(theta33),2)*sin(theta22) + pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*cos(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha2)*cos(alpha3)*sin(alpha1)*sin(theta11)*pow(sin(theta22),2)*pow(sin(theta33),2) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*cos(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha1)*cos(alpha2)*sin(alpha3)*cos(theta22)*cos(theta33)*sin(theta11) + cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta22)*pow(cos(theta33),2)*sin(theta11) + pow(cos(alpha1),2)*cos(alpha2)*sin(alpha3)*cos(theta11)*cos(theta33)*sin(theta22) + cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta22)*sin(theta11)*pow(sin(theta33),2) + cos(alpha2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta11)*cos(theta33)*sin(theta22) - cos(alpha1)*pow(cos(alpha2),2)*sin(alpha3)*sin(theta11)*sin(theta22)*sin(theta33) - sin(alpha1)*sin(alpha2)*sin(alpha3)*pow(cos(theta22),2)*cos(theta33)*sin(theta11) - cos(alpha1)*pow(sin(alpha2),2)*sin(alpha3)*sin(theta11)*sin(theta22)*sin(theta33) - sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta33)*sin(theta11)*pow(sin(theta22),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) + (sin(Rx)*sin(Rz)*(sin(alpha1)*pow(sin(alpha2),2)*sin(alpha3)*sin(theta22)*sin(theta33) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(cos(theta22),2)*pow(cos(theta33),2) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(cos(theta22),2)*pow(sin(theta33),2) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(cos(theta33),2)*pow(sin(theta22),2) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(sin(theta22),2)*pow(sin(theta33),2) - cos(alpha2)*sin(alpha1)*sin(alpha3)*cos(theta22)*cos(theta33) - cos(alpha1)*sin(alpha2)*sin(alpha3)*pow(cos(theta22),2)*cos(theta33) - cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta22)*pow(cos(theta33),2) - cos(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta33)*pow(sin(theta22),2) - cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta22)*pow(sin(theta33),2) + pow(cos(alpha2),2)*sin(alpha1)*sin(alpha3)*sin(theta22)*sin(theta33)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2));
	R31 = ((cos(Ry)*sin(Rx) + cos(Rx)*cos(Rz)*sin(Ry))*(pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*pow(cos(theta33),2)*sin(theta11)*sin(theta22) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(cos(theta22),2)*pow(sin(theta33),2) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(cos(theta33),2)*pow(sin(theta22),2) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(sin(theta22),2)*pow(sin(theta33),2) - cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta11)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*pow(cos(theta33),2)*sin(theta11)*sin(theta22) + pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*sin(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*sin(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta22)*sin(theta11)*sin(theta33) - cos(alpha1)*cos(alpha2)*sin(alpha3)*cos(theta11)*cos(theta22)*cos(theta33) - cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta11)*cos(theta22)*pow(cos(theta33),2) - cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta11)*cos(theta22)*pow(sin(theta33),2) + cos(alpha1)*pow(cos(alpha2),2)*sin(alpha3)*cos(theta11)*sin(theta22)*sin(theta33) + sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta11)*pow(cos(theta22),2)*cos(theta33) + pow(cos(alpha1),2)*cos(alpha2)*sin(alpha3)*cos(theta33)*sin(theta11)*sin(theta22) + cos(alpha1)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta11)*sin(theta22)*sin(theta33) + cos(alpha2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta33)*sin(theta11)*sin(theta22) + sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta11)*cos(theta33)*pow(sin(theta22),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) - ((sin(Ry)*sin(Rx) - cos(Ry)*cos(Rx)*cos(Rz))*(cos(alpha2)*cos(alpha3)*sin(alpha1)*pow(cos(theta22),2)*pow(cos(theta33),2)*sin(theta11) + pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*cos(theta11)*pow(cos(theta33),2)*sin(theta22) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha2)*cos(alpha3)*sin(alpha1)*pow(cos(theta22),2)*sin(theta11)*pow(sin(theta33),2) + cos(alpha2)*cos(alpha3)*sin(alpha1)*pow(cos(theta33),2)*sin(theta11)*pow(sin(theta22),2) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*cos(theta11)*pow(cos(theta33),2)*sin(theta22) + pow(cos(alpha1),2)*cos(alpha3)*sin(alpha2)*cos(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha2)*cos(alpha3)*sin(alpha1)*sin(theta11)*pow(sin(theta22),2)*pow(sin(theta33),2) + cos(alpha3)*pow(sin(alpha1),2)*sin(alpha2)*cos(theta11)*sin(theta22)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*sin(alpha3)*cos(theta11)*cos(theta22)*sin(theta33) + cos(alpha1)*cos(alpha2)*sin(alpha3)*cos(theta22)*cos(theta33)*sin(theta11) + cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta22)*pow(cos(theta33),2)*sin(theta11) + pow(cos(alpha1),2)*cos(alpha2)*sin(alpha3)*cos(theta11)*cos(theta33)*sin(theta22) + cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta22)*sin(theta11)*pow(sin(theta33),2) + cos(alpha2)*pow(sin(alpha1),2)*sin(alpha3)*cos(theta11)*cos(theta33)*sin(theta22) - cos(alpha1)*pow(cos(alpha2),2)*sin(alpha3)*sin(theta11)*sin(theta22)*sin(theta33) - sin(alpha1)*sin(alpha2)*sin(alpha3)*pow(cos(theta22),2)*cos(theta33)*sin(theta11) - cos(alpha1)*pow(sin(alpha2),2)*sin(alpha3)*sin(theta11)*sin(theta22)*sin(theta33) - sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta33)*sin(theta11)*pow(sin(theta22),2)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2)*pow(sin(theta11),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta11),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta11),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta11),2)*pow(sin(theta22),2)*pow(sin(theta33),2)) - (cos(Rx)*sin(Rz)*(sin(alpha1)*pow(sin(alpha2),2)*sin(alpha3)*sin(theta22)*sin(theta33) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(cos(theta22),2)*pow(cos(theta33),2) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(cos(theta22),2)*pow(sin(theta33),2) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(cos(theta33),2)*pow(sin(theta22),2) + cos(alpha1)*cos(alpha2)*cos(alpha3)*pow(sin(theta22),2)*pow(sin(theta33),2) - cos(alpha2)*sin(alpha1)*sin(alpha3)*cos(theta22)*cos(theta33) - cos(alpha1)*sin(alpha2)*sin(alpha3)*pow(cos(theta22),2)*cos(theta33) - cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta22)*pow(cos(theta33),2) - cos(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta33)*pow(sin(theta22),2) - cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta22)*pow(sin(theta33),2) + pow(cos(alpha2),2)*sin(alpha1)*sin(alpha3)*sin(theta22)*sin(theta33)))/(pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(cos(alpha3),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha2),2)*pow(sin(alpha1),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(cos(alpha3),2)*pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(cos(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta22),2)*pow(sin(theta33),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(cos(theta33),2)*pow(sin(theta22),2) + pow(sin(alpha1),2)*pow(sin(alpha2),2)*pow(sin(alpha3),2)*pow(sin(theta22),2)*pow(sin(theta33),2));

	theta44 = atan2(R23,R13);
	theta55 = atan2(sqrt(pow(R13,2)+pow(R23,2)), R33);
	theta66 = atan2(-R32, R31);

	EQ.THETA1 = theta11*180/pi;
	EQ.THETA2 = theta22*180/pi;
	EQ.THETA3 = (theta33+pi/2)*180/pi;
	EQ.THETA4 = theta44*180/pi;
	EQ.THETA5 = theta55*180/pi;
	EQ.THETA6 = (theta66+pi/2)*180/pi;
	Arm_Limit(&EQ.THETA1, &EQ.THETA2, &EQ.THETA3, &EQ.THETA4, &EQ.THETA5, &EQ.THETA6);
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
  MX_UART4_Init();
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

//HOME(); // Call the function Home to use
//////	Initial angle
//theta1 = 55;  // Set_Point as Home
//theta2 = 180;
//theta3 = 0;
//theta4 = 0;
//theta5 = 90;
//Arm.theta1=90;
//Arm.theta2=90;
//Arm.theta3=90;
//Arm.theta4=0;
//Arm.theta5=0;
//
//	Accel_Stepper_Move(&Stepper1, angle2step(35,5), 1000, 1000, 200);
//	Accel_Stepper_Move(&Stepper2, angle2step(-90,20), 1000, 1000, 200);
//	Accel_Stepper_Move(&Stepper3, angle2step(90,20), 1000, 1000, 200);
//	Accel_Stepper_Move(&Stepper4, angle2step(0,14), 1000, 1000, 200);
//	Accel_Stepper_Move(&Stepper5, angle2step(-90,14), 1000, 1000, 200);
//  angle2step(angle move back after get touch with limit switch to Home, ratio);

//CLI_Init(&huart4);
MY_FLASH_SetSectorAddrs(11, 0x080E5000); // address flash memory
//MY_FLASH_SetSectorAddrs(sector, address);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// UART Start
   if (state == 0) HAL_UART_Receive(&huart1, (uint8_t*) &uart_buffer, sizeof(uart_buffer), 1000);
   if (state == 0){
	   HAL_UART_Transmit(&huart1,"OK", sizeof("OK"), 1000);


   }
//    CLI_Process(); // Call the CLI Library to use(Use for cmd from computer only)

	char* msg=strtok(uart_buffer," ");//Get msg and store by uart_buffer
	if(strcmp(msg, "home\r\n")==0){

//After get msg string compare start comparing the value (msg and set_home); (*Logic AND)
//The both of two value should the same by  using string compare

    HOME(); // Call the function Home to use

//	Initial angle
    theta1 = 55;  // Set_Point as Home
    theta2 = 180;
    theta3 = 0;
    theta4 = 0;
    theta5 = 90;
    Arm.theta1=90;
    Arm.theta2=90;
    Arm.theta3=90;
    Arm.theta4=0;
    Arm.theta5=0;
  // Arm_Limit(&Arm.theta1, &Arm.theta2, &Arm.theta3, &Arm.theta4, &Arm.theta5, &Arm.theta6);
step1=angle2step(Arm.theta1-theta1,5);
step2=angle2step(Arm.theta2-theta2,20);
step3=angle2step(Arm.theta3-theta3,20);
step4=angle2step(Arm.theta4-theta4,14);
step5=angle2step(Arm.theta5-theta5,14);
theta1=Arm.theta1;
theta2=Arm.theta2;
theta3=Arm.theta3;
theta4=Arm.theta4;
theta5=Arm.theta5;

Syn_Stepper(step1,step2,step3,step4,step5,step6);
memset(uart_buffer, 0, 100);
//CLI_Printf("Done!");
	}
	  else if (strcmp(msg, "set_amax")==0){
	      char* cmd=strtok(uart_buffer, " ");
	   if (cmd !=NULL){
	    char* arg = strtok(NULL, " "); // read the string after x (read char from cmd)
	    uint32_t val = atoi(arg);    // Read ascii computer Language
	    memcpy(&memory.a_max, &val, sizeof(uint32_t));
	    memset(uart_buffer, 0, 100);
//	           memset (void *, int, size_t); //Reset cmd that store by FLASH MEMORIE
	    	                                //In this use to store the value of a_max; v_max and theta read from cmd
	   }
	  }
	  else  if (strcmp(msg,"set_vmax")==0){
		   char* cmd=strtok(uart_buffer," ");
	   if (cmd!=NULL){
		char* arg = strtok(NULL, " "); //Read cmd form UART to clear value one by one  // Read cmd after x
		uint32_t val = atoi(arg);
		memcpy(&memory.v_max, &val, sizeof(uint32_t));
		memset(uart_buffer, 0, 100);
	     }
	   }
	  else if (strcmp(msg,"theta")==0){
	  if (msg !=NULL){

		   char* arg1=strtok(NULL," ");
		   char* arg2=strtok(NULL," ");
		   char* arg3=strtok(NULL," ");
		   char* arg4=strtok(NULL," ");
		   char* arg5=strtok(NULL," ");
		   char* arg6=strtok(NULL," ");
		   theta.theta1 = atof(arg1);
		   theta.theta2 = atof(arg2);
		   theta.theta3 = atof(arg3);
		   theta.theta4 = atof(arg4);
		   theta.theta5 = atof(arg5);
		   theta.theta6 = atof(arg6);
		   step1 = angle2step(theta.theta1-theta1, 5);//angle2step(angle,ratio of gear box motor)
		   step2 = angle2step(theta.theta2-theta2,20);
		   step3 = angle2step(theta.theta3-theta3,20);
		   step4 = angle2step(theta.theta4-theta4,14);
		   step5 = angle2step(theta.theta5-theta5,14);
		   step6 = angle2step(theta.theta6-theta6,14);
		   theta1 = theta.theta1; // Set theta to update
		   theta2 = theta.theta2;
		   theta3 = theta.theta3;
		   theta4 = theta.theta4;
		   theta5 = theta.theta5;
		   theta6 = theta.theta6;

		  Syn_Stepper(step1, step2,step3,step4,step5,step6);
		  memset(uart_buffer, 0, 100); // when memset => int=o;
	     }
    }
	  else if (strcmp(msg, "xyz_RxRyRz")==0){
	  	if (msg !=NULL){

	  	char* arg1=strtok(NULL, " ");
	  	char* arg2=strtok(NULL, " ");
	  	char* arg3=strtok(NULL, " ");
	  	char* arg4=strtok(NULL, " ");
	  	char* arg5=strtok(NULL, " ");
	  	char* arg6=strtok(NULL, " ");

	  	 setpos.x = atof(arg1);
	  	 setpos.y = atof(arg2);
	  	 setpos.z = atof(arg3);
	  	 setpos.Rx= atof(arg4);
	  	 setpos.Ry= atof(arg5);
	  	 setpos.Rz= atof(arg6);
//	 	 Inverse_6Joint(setpos.x,setpos.y,setpos.z);
	 	 Inverse_6Joint(setpos.x,setpos.y,setpos.z,setpos.Rx,setpos.Ry,setpos.Rz);

	  	   step1=angle2step(EQ.THETA1-theta1, 5);
	  	   step2=angle2step(EQ.THETA2-theta2, 20);
	  	   step3=angle2step(EQ.THETA3-theta3, 20);
	  	   step4=angle2step(EQ.THETA4-theta4, 14);
	  	   step5=angle2step(EQ.THETA5-theta5, 14);
	  	   step6=angle2step(EQ.THETA6-theta6, 14);

	  	   theta1 = EQ.THETA1; // Set theta to update
	  	   theta2 = EQ.THETA2;
	  	   theta3 = EQ.THETA3;
	  	   theta4 = EQ.THETA4;
	  	   theta5 = EQ.THETA5;
	  	   theta6 = EQ.THETA6;

	  	 Syn_Stepper(step1,step2,step3,step4,step5,step6);
	      memset(uart_buffer, 0, 100);
	  	}
	    }


	   if(Stepper1.run_status == 0){
	 		    Accel_Stepper_Move(&Stepper1, step1 , accel1, accel1, theta1dot);
	 		    step1=0;
	 	        }
	 	if(Stepper2.run_status == 0){
	 		    Accel_Stepper_Move(&Stepper2, step2 , accel2, accel2, theta2dot);
	 		    step2=0;
	 	        }
	 	if(Stepper3.run_status == 0){
	 			Accel_Stepper_Move(&Stepper3, step3 , accel3, accel3, theta3dot);
	 			step3=0;
	 	        }
	 	if(Stepper4.run_status == 0){
	 				Accel_Stepper_Move(&Stepper4, step4 , accel4, accel4, theta4dot);
	 			step4=0;
	 			}
	 	if(Stepper5.run_status == 0){
	 			Accel_Stepper_Move(&Stepper5, step5 , accel5, accel5, theta5dot);
	 			step5=0;
	 			}
	   if(Stepper6.run_status == 0){
	 	        Accel_Stepper_Move(&Stepper6, step6 , accel6, accel6, theta6dot);
	 			step6=0;
	 			}
	HAL_Delay(100); // Time day for read the value of xyz;
	memset(uart_buffer, 0, 100);
	   }
// UART END
//--------------------------------------------------------------
//	   step  =angle2step(angle, ratio of gear box motor);
//	   step1 = angle2step(theta.theta1-theta1, 5);
//	   step2 = angle2step(theta.theta2-theta2,20);
//	   step3 = angle2step(theta.theta3-theta3,20);
//	   step4 = angle2step(theta.theta4-theta4,14);
//	   step5 = angle2step(theta.theta5-theta5,14);
//	   step6 = angle2step(theta.theta6-theta6,14);

//  if(Stepper1.run_status == 0){
//		    Accel_Stepper_Move(&Stepper1, step1 , accel1, accel1, theta1dot);// acceleration=deceleration
////	    Accel_Stepper_Move(&Stepper1, step1 , accel1, decel1, speed);
////	    theta1 = theta.theta1;
//	        step1=0;
//	  }
//	if(Stepper2.run_status == 0){
//		    Accel_Stepper_Move(&Stepper2, step2 , accel2, accel2, theta2dot);
////		theta2 = theta.theta2;
//		    step2=0;
//	}
//	if(Stepper3.run_status == 0){
//			Accel_Stepper_Move(&Stepper3, step3 , accel3, accel3, theta3dot);
////		theta3 = theta.theta3;
//			step3=0;
//	}
//	if(Stepper4.run_status == 0){
//			Accel_Stepper_Move(&Stepper4, step4 , accel4, accel4, theta4dot);
////		theta4 = theta.theta4;
//			step4=0;
//		}
//	if(Stepper5.run_status == 0){
//			Accel_Stepper_Move(&Stepper5, step5 , accel5, accel5, theta5dot);
////		theta5 = theta.theta5;
//			step5=0;
//		}
//	if(Stepper6.run_status == 0){
//			Accel_Stepper_Move(&Stepper6, step6 , accel6, accel6, theta6dot);
////		theta6 = theta.theta6;
//			step6=0;
//		}
//	HAL_Delay(100);
//	limit_switch (); // Call the limit switch to test
//  }

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

