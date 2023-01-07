/*
 * EncoderMotor.h
 *
 *  Created on: May 23, 2022
 *      Author:
 */

#ifndef INC_ENCODERMOTOR_H_
#define INC_ENCODERMOTOR_H_

#include "stdint.h"

#define pi 3.14159f

#define PID_Input 6 //number of input using PID
#define Motor1 0
#define Motor2 1
#define Motor3 2
#define Motor4 3
#define Motor5 4
#define Motor6 5

extern uint8_t nowA[6];
extern uint8_t nowB[6];
extern uint16_t cnt[6];
extern uint8_t lastA[6];
extern uint8_t lastB[6];
extern uint32_t Enc_count[6];
extern uint8_t dir[6];

uint32_t count[5]; //count pulse from encoder
uint32_t new_count[5];
uint32_t diff[6]; //difference between count and new_count in a sample time

uint8_t count_state[5];
float speed[5];
float rdps[5];
float kmph[5];

float Motor_Speed(int j, float SampleTime, float N_round);
uint32_t encoder(int i);

#endif /* INC_ENCODERMOTOR_H_ */
