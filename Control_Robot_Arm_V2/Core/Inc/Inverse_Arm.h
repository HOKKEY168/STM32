/*
 * Inverse_Arm.h
 *
 *  Created on: Apr 29, 2022
 *      Author: PHANNY
 */

#ifndef INC_INVERSE_ARM_H_
#define INC_INVERSE_ARM_H_
#include "math.h"

#define a1 0.0f
#define a2 305.0f //mm
#define a3 280.0f
#define d1 0.0f
#define d5 210.0f

typedef struct{
	float theta1,theta2,theta3,theta4,theta5,theta6;

}Inv_Arm_t;
void cal_theta(float x, float y, float z);
#endif /* INC_INVERSE_ARM_H_ */
