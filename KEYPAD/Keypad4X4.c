/*
 * Keypad4X4.c
 *
 *  Created on: August 29, 2023
 *  Author	  : HOKKEY
 *  This is an STM32 device driver library for the Keypad4X4, using STM HAL libraries
 *  Copyright (C) 2023 HOKKEY
 */

#include "Keypad4X4.h"
char key;

char Get_Key(void){
	while(1)
	{
	//COLUMN 4
	HAL_GPIO_WritePin (C1_GPIO_Port, C1_Pin, 1);   // Pull the C1 HIGH
	HAL_GPIO_WritePin (C2_GPIO_Port, C2_Pin, 0);   // Pull the C2 LOW
	HAL_GPIO_WritePin (C3_GPIO_Port, C3_Pin, 0);   // Pull the C3 LOW
	HAL_GPIO_WritePin (C4_GPIO_Port, C4_Pin, 0);   // Pull the C4 LOW

	if ((HAL_GPIO_ReadPin (R1_GPIO_Port, R1_Pin)))   // if  R1 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R1_GPIO_Port, R1_Pin)));   // wait till the button is pressed
		key='1';
		break;
	}

	if ((HAL_GPIO_ReadPin (R2_GPIO_Port, R2_Pin)))   // if R2 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R2_GPIO_Port, R2_Pin)));   // wait till the button is pressed
		key='4';
		break;
	}

	if ((HAL_GPIO_ReadPin (R3_GPIO_Port, R3_Pin)))   // if R3 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R3_GPIO_Port, R3_Pin)));   // wait till the button is pressed
		key='7';
		break;
	}
	if ((HAL_GPIO_ReadPin (R4_GPIO_Port, R4_Pin)))   // if R4 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R4_GPIO_Port, R4_Pin)));   // wait till the button is pressed
		key='*';
		break;
	}

	//COLUMN 3
	HAL_GPIO_WritePin (C1_GPIO_Port, C1_Pin, 0);    // Pull the C1 HIGH
	HAL_GPIO_WritePin (C2_GPIO_Port, C2_Pin, 1);    // Pull the C2 LOW
	HAL_GPIO_WritePin (C3_GPIO_Port, C3_Pin, 0);    // Pull the C3 LOW
	HAL_GPIO_WritePin (C4_GPIO_Port, C4_Pin, 0);    // Pull the C4 LOW

	if ((HAL_GPIO_ReadPin (R1_GPIO_Port, R1_Pin)))   // if  R1 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R1_GPIO_Port, R1_Pin)));   // wait till the button is pressed
		key='2';
		break;
	}

	if ((HAL_GPIO_ReadPin (R2_GPIO_Port, R2_Pin)))   // if R2 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R2_GPIO_Port, R2_Pin)));   // wait till the button is pressed
		key='5';
		break;
	}

	if ((HAL_GPIO_ReadPin (R3_GPIO_Port, R3_Pin)))   // if R3 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R3_GPIO_Port, R3_Pin)));   // wait till the button is pressed
		key='8';
		break;
	}
	if ((HAL_GPIO_ReadPin (R4_GPIO_Port, R4_Pin)))   // if R4 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R4_GPIO_Port, R4_Pin)));   // wait till the button is pressed
		key='0';
		break;
	}
	//COLUMN 2
	HAL_GPIO_WritePin (C1_GPIO_Port, C1_Pin, 0);     // Pull the C1 LOW
	HAL_GPIO_WritePin (C2_GPIO_Port, C2_Pin, 0);   // Pull the C2 LOW
	HAL_GPIO_WritePin (C3_GPIO_Port, C3_Pin, 1);   // Pull the C3 HIGH
	HAL_GPIO_WritePin (C4_GPIO_Port, C4_Pin, 0);   // Pull the C4 LOW

	if ((HAL_GPIO_ReadPin (R1_GPIO_Port, R1_Pin)))   // if  R1 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R1_GPIO_Port, R1_Pin)));   // wait till the button is pressed
		key='3';
		break;
	}

	if ((HAL_GPIO_ReadPin (R2_GPIO_Port, R2_Pin)))   // if R2 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R2_GPIO_Port, R2_Pin)));   // wait till the button is pressed
		key='6';
		break;
	}

	if ((HAL_GPIO_ReadPin (R3_GPIO_Port, R3_Pin)))   // if R3 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R3_GPIO_Port, R3_Pin)));   // wait till the button is pressed
		key='9';
		break;
	}
	if ((HAL_GPIO_ReadPin (R4_GPIO_Port, R4_Pin)))   // if R4 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R4_GPIO_Port, R4_Pin)));   // wait till the button is pressed
		key='#';
		break;
	}
	//COLUMN 1
	HAL_GPIO_WritePin (C1_GPIO_Port, C1_Pin, 0);    // Pull the C1 LOW
	HAL_GPIO_WritePin (C2_GPIO_Port, C2_Pin, 0);   // Pull the C2 LOW
	HAL_GPIO_WritePin (C3_GPIO_Port, C3_Pin, 0);   // Pull the C3 LOW
	HAL_GPIO_WritePin (C4_GPIO_Port, C4_Pin, 1);     // Pull the C4 HIGH

	if ((HAL_GPIO_ReadPin (R1_GPIO_Port, R1_Pin)))   // if  R1 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R1_GPIO_Port, R1_Pin)));   // wait till the button is pressed
		key='A';
		break;
	}

	if ((HAL_GPIO_ReadPin (R2_GPIO_Port, R2_Pin)))   // if R2 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R2_GPIO_Port, R2_Pin)));   // wait till the button is pressed
		key='B';
		break;
	}

	if ((HAL_GPIO_ReadPin (R3_GPIO_Port, R3_Pin)))   // if R3 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R3_GPIO_Port, R3_Pin)));   // wait till the button is pressed
		key='C';
		break;
	}
	if ((HAL_GPIO_ReadPin (R4_GPIO_Port, R4_Pin)))   // if R4 is HIGH
	{
		while ((HAL_GPIO_ReadPin (R4_GPIO_Port, R4_Pin)));   // wait till the button is pressed
		key='D';
		break;
	}
}
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Black Pill
	return(key);
}

//ROW(input)-> Pull Up or Pull Down
//COLUMN(output)
