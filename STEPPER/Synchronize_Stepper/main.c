/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : combind all stepper satrt and stop and the same time
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
//Formula compare the value of stepper
#define max(a,b) (a > b) ? a : b
#define min(a,b) (a > b) ? b : a

// Compare the value of 6 stepper motor
int max6(int a, int b, int c, int d,int e, int f){
	int n1=max(a,b);
	int n2=max(c,d);
	int n3=max(e,f);
	int n4=max(n1,n2);
    return max(n3,n4);
}
void Syn_Stepper(int step1, int step2, int step3, int step4, int step5, int step6){
	int step_max = max6(abs(step1), abs(step2), abs(step3), abs(step4), abs(step5), abs(step6));
	float coef1 = fabs(step1)/step_max;
	float coef2 = fabs(step2)/step_max;
	float coef3 = fabs(step3)/step_max;
	float coef4 = fabs(step4)/step_max;
	float coef5 = fabs(step5)/step_max;
	float coef6 = fabs(step6)/step_max;
	theta1dot = max_speed * coef1;  //Let's theta..dot= rpm or speed
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

