/*
 * KinematicEquation.h
 *
 *  Created on: May 23, 2022
 *      Author:
 */

#ifndef INC_KINEMATICEQUATION_H_
#define INC_KINEMATICEQUATION_H_

#define r 6.35f //radius for Omni Wheel

float KinematicM1(float vx, float vy, float omega);
float KinematicM2(float vx, float vy, float omega);
float KinematicM3(float vx, float vy, float omega);
float KinematicM4(float vx, float vy, float omega);

#endif /* INC_KINEMATICEQUATION_H_ */
