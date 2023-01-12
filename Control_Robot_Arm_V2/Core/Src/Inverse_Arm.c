/*
 * Inverse_Arm.c
 *
 *  Created on: Apr 29, 2022
 *      Author: PHANNY
 */
#include "Inverse_Arm.h"

extern Inv_Arm_t Arm;

void cal_theta(float x, float y, float z){
//	float x = r*cosf(theta);
//	float y = r*sinf(theta);

	Arm.theta1 = atan2f(y,x);
	float xc = x - d5*cosf(Arm.theta1);
	float yc = y - d5*sinf(Arm.theta1);

//	D = (xc^2+yc^2+(zc-d1)^2-a2^2-a3^2)/(2*a2*a3);
//	th3 = atan2(-sqrt(abs(1-D^2)),D);
//	h1 =atan2(a3*sin(th3),a2+(a3*cos(th3)));
//	h2 = atan2(zc-d1,sqrt(xc^2+yc^2));
//	th2 = h2-h1;

	float D = (xc*xc+yc*yc+(z-d1)*(z-d1)-a2*a2-a3*a3)/(2*a2*a3);
	Arm.theta3 = atan2f(-sqrtf(1-(D*D)), D);

	float h1 = atan2f(a3*sinf(Arm.theta3),a2 + (a3*cosf(Arm.theta3)));
	float h2 = atan2f(z-d1,sqrtf(xc*xc+yc*yc));
	Arm.theta2 = h2-h1;
//	Arm.theta4 = -Arm.theta2 - Arm.theta3;
}
