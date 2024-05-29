/*
 * Save_FlashMemory.c
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
#include "Save_FlashMemory.h"

//Private variables
//1. sector start address
static uint32_t MY_SectorAddrs;
static uint8_t MY_SectorNum;

//functions definitions
//1. Erase Sector
static void MY_FLASH_EraseSector(void)
{
	HAL_FLASH_Unlock();
	//Erase the required Flash sector
	FLASH_Erase_Sector(MY_SectorNum, FLASH_VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();
}

//2. Set Sector Adress
void MY_FLASH_SetSectorAddrs(uint8_t sector, uint32_t addrs)
{
	MY_SectorNum = sector;
	MY_SectorAddrs = addrs;
}

//3. Write Flash
void MY_FLASH_WriteN(uint32_t idx,uint8_t sector, void *wrBuf, uint32_t Nsize, DataTypeDef dataType)
{
//	MY_FLASH_EraseSector();  // Erase sector before write
//	uint32_t flashAddress = MY_SectorAddrs | idx; // Ensure offset for a byte
//	uint32_t flashAddress = MY_SectorAddrs + idx * 4; // Ensure offset is in bytes
	uint32_t flashAddress = MY_SectorAddrs + idx * (dataType == DATA_TYPE_32 || dataType == DATA_TYPE_FLOAT ? 4 : (dataType == DATA_TYPE_16 ? 2 : 1));
	//Erase sector before write
	MY_FLASH_EraseSector();
	//Unlock Flash
	HAL_FLASH_Unlock();
//	HAL_FLASH_OB_Unlock();
	//Write to Flash
	switch(dataType)
	{
		case DATA_TYPE_8:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddress , ((uint8_t *)wrBuf)[i]);
					flashAddress++;
				}
			break;

		case DATA_TYPE_16:
				for(uint16_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flashAddress , ((uint16_t *)wrBuf)[i]);
					flashAddress+=2;
				}
			break;

		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress , ((uint32_t *)wrBuf+i));
					flashAddress+=4;
				}
			break;
//		case DATA_TYPE_32:
		case DATA_TYPE_FLOAT:
				for(uint32_t i=0; i<Nsize; i++)
				{
//					uint32_t data;
//					memcpy(&data, (float *)wrBuf + i, sizeof(uint32_t));
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress , *(uint32_t *)((float *)wrBuf+i));
					flashAddress+=4;
				}
			break;
	}
	//Lock the Flash space
	HAL_FLASH_Lock();
//	HAL_FLASH_OB_Lock();
}
//4. Read Flash
void MY_FLASH_ReadN(uint32_t idx, uint8_t sector, void *rdBuf, uint32_t Nsize, DataTypeDef dataType)
{
//	uint32_t flashAddress = MY_SectorAddrs | idx;
//	uint32_t flashAddress = MY_SectorAddrs + idx * 4; // Ensure offset is in bytes
	uint32_t flashAddress = MY_SectorAddrs + idx * (dataType == DATA_TYPE_32 || dataType == DATA_TYPE_FLOAT ? 4 : (dataType == DATA_TYPE_16 ? 2 : 1));
	switch(dataType)
	{
		case DATA_TYPE_8:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint8_t *)rdBuf + i) = *(uint8_t *)flashAddress;
					flashAddress++;
				}
			break;

		case DATA_TYPE_16:
				for(uint16_t i=0; i<Nsize; i++)
				{
					*((uint16_t *)rdBuf + i) = *(uint16_t *)flashAddress;
					flashAddress+=2;
				}
			break;

		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint32_t *)rdBuf + i) = *(uint32_t *)flashAddress;
					flashAddress+=4;
				}
			break;
//		case DATA_TYPE_32:
		case DATA_TYPE_FLOAT:
				for(uint32_t i=0; i<Nsize; i++)
				{
					 uint32_t data = *(uint32_t *)flashAddress;
					*((float *)rdBuf + i) = *(float *)flashAddress;
					flashAddress+=4;
				}
			break;
	}
}


