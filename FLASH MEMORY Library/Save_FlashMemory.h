/*
 * Save_FlashMemory.h
 *
 *  Created on: May 23, 2024
 *      Author: HOKKEY
 *  Library	  : STM32F40x Internal FLASH Read/Write
 *  MY_FLASH library implements the following basic functionalities
	- Set sectos address
	- Flash Sector Erase
	- Flash Write
    - Flash Read
 */

#ifndef INC_SAVE_FLASHMEMORY_H_
#define INC_SAVE_FLASHMEMORY_H_


#include "stm32f4xx_hal.h"

//Typedefs
#define MAX_TARGETS 10
#define NUM_PARAMETERS 6
//1. data size
typedef enum
{
	DATA_TYPE_8=0,
	DATA_TYPE_16,
	DATA_TYPE_32,
	DATA_TYPE_FLOAT,
}DataTypeDef;

typedef struct{
	uint16_t j_max;
	uint16_t a_max;
	uint16_t v_max;
//	float pos_x;
//	float pos_y;
//	float pos_z;
//	float orient_Rx;
//	float orient_Ry;
//	float orient_Rz;
	float Target[MAX_TARGETS][NUM_PARAMETERS];

}Memory_t;

//functions prototypes
//1. Erase Sector
//static void MY_FLASH_EraseSector(void);

//2. Set Sector Adress
void MY_FLASH_SetSectorAddrs(uint8_t sector, uint32_t addrs);
//3. Write Flash
void MY_FLASH_WriteN(uint32_t idx,uint8_t sector, void *wrBuf, uint32_t Nsize, DataTypeDef dataType);
//4. Read Flash
void MY_FLASH_ReadN(uint32_t idx,uint8_t sector, void *rdBuf, uint32_t Nsize, DataTypeDef dataType);



#endif /* INC_SAVE_FLASHMEMORY_H_ */
