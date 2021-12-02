/*
 * FLASH_SECTOR_F4.h
 *
 *  Created on: 15 nov. 2021
 *      Author: psimo
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "stdint.h"

#define PHOTO_ADDR 0x08010000

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint8_t *Data, uint16_t numberofwords);

void Flash_Read_Data (uint32_t StartSectorAddress, uint8_t *RxBuf, uint16_t numberofwords);

void Convert_To_Str (uint32_t *Data, char *Buf);

void Flash_Write_NUM (uint32_t StartSectorAddress, float Num);

float Flash_Read_NUM (uint32_t StartSectorAddress);


#endif /* INC_FLASH_H_ */
