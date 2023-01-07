/*
 * Encoderlib.h
 *
 *  Created on: 7 Jun 2022
 *      Author: Thea
 */

#ifndef INC_ENCODER_RO_H_
#define INC_ENCODER_RO_H_
#include "stdint.h"



typedef struct{
	uint8_t nowA_Ro;
	uint8_t nowB_Ro;
	uint16_t cnt_Ro;
	uint8_t lastA_Ro;
	uint8_t lastB_Ro;
	uint32_t Enc_count_Ro;
	uint8_t dir_Ro;
}ENCODER_t;

//typedef struct{
//	uint32_t new_count_Ro=0;
//	uint32_t diff_Ro = 0;
//	float speed_Ro = 0;
//	uint32_t Enc_count_Ro = 0;
//	float rdps_Ro =0;
//}SPEED_t;

void Encoder_Init(ENCODER_t* encoder_ro);
uint32_t Enc_getCount(ENCODER_t* encoder_ro);
//void Motor_Speed_RPM_Ro_Init(SPEED_t* motor_rpm);

#endif /* INC_ENCODER_RO_H_ */
