/*!
 * \file      flash.c
 *
 * \brief     It contains all the functions to read from / write in the flash memory
 * 			  It also contains the different addresses of all the variables
 *
 *
 * \created on: 15/11/2021
 *
 * \author    Pol Simon
 *
 * \author    David Reiss
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include <stdint.h>
#include <stdbool.h>

#define PHOTO_ADDR 					0x08020000
#define PAYLOAD_STATE_ADDR 			0x08008000
#define COMMS_STATE_ADDR 			0x08008001
#define DEPLOYMENT_STATE_ADDR 		0x08008002
#define DEPLOYMENTRF_STATE_ADDR 	0x08008003
#define DETUMBLE_STATE_ADDR 		0x08008004
#define KP_ADDR 					0x08008005
#define SF_ADDR 					0x08008006
#define CRC_ADDR 					0x08008007
#define COUNT_PACKET_ADDR 			0x08008008
#define COUNT_WINDOW_ADDR 			0x08008009
#define COUNT_RTX_ADDR 				0x08008010
#define PHOTO_RESOL_ADDR 			0x08008011
#define PHOTO_COMPRESSION_ADDR 		0x08008012
#define F_MIN_ADDR 					0x08008013
#define F_MAX_ADDR 					0x08008015
#define DELTA_F_ADDR 				0x08008017
#define INTEGRATION_TIME_ADDR 		0x08008019

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint8_t *Data, uint16_t numberofbytes);

void Write_Flash(uint32_t StartSectorAddress, uint8_t *Data, uint16_t numberofbytes);

void Flash_Read_Data (uint32_t StartSectorAddress, uint8_t *RxBuf, uint16_t numberofbytes);

void Check_Redundancy(uint32_t Address, uint8_t *RxDef, uint16_t numberofbytes);

void Read_Flash(uint32_t StartSectorAddress, uint8_t *RxBuf, uint16_t numberofbytes);

#endif /* INC_FLASH_H_ */
