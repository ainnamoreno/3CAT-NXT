/*
 * telecommands.c
 *
 *  Created on: Dec 11, 2021
 *      Author: psimo
 */

#include "telecommands.h"


/**************************************************************************************
 *                                                                                    *
 * Function:  process_telecommand                                                     *
 * --------------------                                                               *
 * processes the information contained in the packet depending on the telecommand     *
 * received																	          *
 *                                                                                    *
 *  header: number of telecommand			                                          *
 *  info: information contained in the received packet								  *
 *                                                                                    *
 *  returns: nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void process_telecommand(uint8_t header, uint8_t info) {
	switch(header) {
	case RESET2:
		HAL_NVIC_SystemReset();
		break;
//	case NOMINAL:
//
//		break;
//	case LOW:
//
//		break;
//	case CRITICAL:
//
//		break;
	case SET_CONSTANT_KP:
		Write_Flash(KP_ADDR, info, 1);
		break;
	case TLE:

		break;
	case SENDDATA:

		break;
	case SENDTELEMETRY:

		break;
	case STOPSENDINGDATA:

		break;
	case ACKDATA:

		break;
	case SET_SF:
		Write_Flash(SF_ADDR, info, 1);
		break;
	case SET_CRC:
		Write_Flash(CRC_ADDR, info, 1);
		break;
	case TAKEPHOTO:
		/*GUARDAR TEMPS FOTO?*/

		break;
	case SET_PHOTO_RESOL:
		Write_Flash(PHOTO_RESOL_ADDR, info, 1);
		break;
	case PHOTO_COMPRESSION:
		Write_Flash(PHOTO_COMPRESSION_ADDR, info, 1);
		break;
	case TAKERF:

		break;
	case F_MIN:
		Write_Flash(F_MIN_ADDR, info, 2);
		break;
	case F_MAX:
		Write_Flash(F_MAX_ADDR, info, 2);
		break;
	case DELTA_F:
		Write_Flash(DELTA_F_ADDR, info, 2);
		break;
	case INTEGRATION_TIME:
		Write_Flash(INTEGRATION_TIME_ADDR, info, 1);
		break;
	case SEND_CONFIG:

		break;
	}
}
