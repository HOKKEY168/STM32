/*
 * EncoderMotor.c
 *
 *  Created on: May 23, 2022
 *      Author:
 */

#include"EncoderMotor.h"

float Motor_Speed(int j, float SampleTime, float N_round) {
	/* j is defined the motor
	 * N_round is the count which the encoder counts in one round
	 */
	new_count[Motor1] = Enc_count[0]; //Motor1
	new_count[Motor2] = Enc_count[1]; //Motor2
	new_count[Motor3] = Enc_count[2]; //Motor3
	new_count[Motor4] = Enc_count[3]; //Motor4
	new_count[Motor5] = Enc_count[4]; // for shooting motor

	count_state[Motor1] = !dir[0];	// Check rotation direction of Motor 1
	count_state[Motor2] = !dir[1];	// Check rotation direction of Motor 2
	count_state[Motor3] = !dir[2];	// Check rotation direction of Motor 3
	count_state[Motor4] = !dir[3];	// Check rotation direction of Motor 4
	count_state[Motor5] = !dir[4];

	if (count_state[j] == 1) {
		if (new_count[j] <= count[j]) { // Check for counter underflow
			diff[j] = count[j] - new_count[j];
		} else {
			diff[j] = (65536 - new_count[j]) + count[j];
		}
		speed[j] = (float) diff[j] * 1000.0f / (N_round * SampleTime) * -1.0;
	} else {
		if (new_count[j] >= count[j]) { // Check for counter overflow
			diff[j] = new_count[j] - count[j];
		} else {
			diff[j] = (65536 - count[j]) + new_count[j];
		}
		speed[j] = (float) diff[j] * 1000.0f / (N_round * SampleTime);
	}
	rdps[j] = (float) 2 * pi * speed[j];
	count[j] = new_count[j];
	kmph[j] = 0.1885 * 0.11 * 9.554 * rdps[j];
	return rdps[j];
}
uint32_t encoder(int i) {
	if (nowA[i] != lastA[i]) {
		lastA[i] = nowA[i];
		if (lastA[i] == 0) {
			if (nowB[i] == 0) {
				dir[i] = 0;
				cnt[i]--;
			} else {
				dir[i] = 1;
				cnt[i]++;
			}
		} else {
			if (nowB[i] == 1) {
				dir[i] = 0;
				cnt[i]--;
			} else {
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	if (nowB[i] != lastB[i]) {
		lastB[i] = nowB[i];
		if (lastB[i] == 0) {
			if (nowA[i] == 1) {
				dir[i] = 0;
				cnt[i]--;
			} else {
				dir[i] = 1;
				cnt[i]++;
			}
		} else {
			if (nowA[i] == 0) {
				dir[i] = 0;
				cnt[i]--;
			} else {
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	return cnt[i];
}

