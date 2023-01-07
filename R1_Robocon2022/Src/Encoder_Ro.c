/*
 * Encoderlib.c
 *
 *  Created on: 7 Jun 2022
 *      Author: Thea
 */

#include "Encoder_Ro.h"

void Encoder_Init(ENCODER_t *encoder_ro) {
	encoder_ro->Enc_count_Ro = 0;
	encoder_ro->cnt_Ro = 0;
	encoder_ro->dir_Ro = 0;
	encoder_ro->lastA_Ro = 0;
	encoder_ro->lastB_Ro = 0;
	encoder_ro->nowA_Ro = 0;
	encoder_ro->nowB_Ro = 0;

}

//void Motor_Speed_RPM_Ro_Init(SPEED_t* motor_rpm){
//	motor_rpm-> new_count_Ro = 0;
//	motor_rpm-> diff_Ro = 0;
//	motor_rpm-> speed_Ro = 0;
//	motor_rpm-> Enc_count_Ro = 0;
//	motor_rpm-> rdps_Ro =0;
//}
//float Motor_Speed_Ro(SPEED_t* motor_rpm, float SampleTime_Ro, float k_round) {
//	/* k is defined the motor
//	 * k_round is the count which the encoder counts in one round
//	 */
//	new_count = Enc_count_Ro; //Motor1
//
//	count_state[Motor1] = !dir[0];	// Check rotation direction of Motor 1
//
//	if (count_state[j] == 1) {
//		if (new_count[j] <= count[j]) { // Check for counter underflow
//			diff[j] = count[j] - new_count[j];
//		} else {
//			diff[j] = (65536 - new_count[j]) + count[j];
//		}
//		speed[j] = (float) diff[j] * 1000.0f / (N_round * SampleTime) * -1.0;
//	} else {
//		if (new_count[j] >= count[j]) { // Check for counter overflow
//			diff[j] = new_count[j] - count[j];
//		} else {
//			diff[j] = (65536 - count[j]) + new_count[j];
//		}
//		speed[j] = (float) diff[j] * 1000.0f / (N_round * SampleTime);
//	}
//	rdps[j] = (float) 2 * pi * speed[j];
//	count[j] = new_count[j];
//	kmph[j] = 0.1885 * 0.11 * 9.554 * rdps[j];
//	return rdps[j];
//}
uint32_t Enc_getCount(ENCODER_t *encoder_ro) {
	if (encoder_ro->nowA_Ro != encoder_ro->lastA_Ro) {
		encoder_ro->lastA_Ro = encoder_ro->nowA_Ro;
		if (encoder_ro->lastA_Ro == 0) {
			if (encoder_ro->nowB_Ro == 0) {
				encoder_ro->dir_Ro = 0;
				encoder_ro->cnt_Ro--;
			} else {
				encoder_ro->dir_Ro = 1;
				encoder_ro->cnt_Ro++;
			}
		} else {
			if (encoder_ro->nowB_Ro == 1) {
				encoder_ro->dir_Ro = 0;
				encoder_ro->cnt_Ro--;
			} else {
				encoder_ro->dir_Ro = 1;
				encoder_ro->cnt_Ro++;
			}
		}
	}
	if (encoder_ro->nowB_Ro != encoder_ro->lastB_Ro) {
		encoder_ro->lastB_Ro = encoder_ro->nowB_Ro;
		if (encoder_ro->lastB_Ro == 0) {
			if (encoder_ro->nowA_Ro == 1) {
				encoder_ro->dir_Ro = 0;
				encoder_ro->cnt_Ro--;
			} else {
				encoder_ro->dir_Ro = 1;
				encoder_ro->cnt_Ro++;
			}
		} else {
			if (encoder_ro->nowA_Ro == 0) {
				encoder_ro->dir_Ro = 0;
				encoder_ro->cnt_Ro--;
			} else {
				encoder_ro->dir_Ro = 1;
				encoder_ro->cnt_Ro++;
			}
		}
	}
	return encoder_ro->cnt_Ro;
}
