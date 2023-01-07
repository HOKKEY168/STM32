/*
 * KinematicEquation.c
 *
 *  Created on: May 23, 2022
 *      Author:
 */

#include "KinematicEquation.h"

/* for Omni wheel*/

float KinematicM1(float vx, float vy, float omega) {
	return (float) (-vy * 0.7 / r - vx * 0.7 / r - omega);
}

float KinematicM2(float vx, float vy, float omega) {
	return (float) (-vy * 0.7 / r + vx * (0.7) / r - omega);
}
float KinematicM3(float vx, float vy, float omega) {
	return (float) (vy * 0.7 / r + vx * 0.7 / r - omega);
}
float KinematicM4(float vx, float vy, float omega) {
	return (float) (vy * 0.7 / r - vx * 0.7 / r - omega);
}

