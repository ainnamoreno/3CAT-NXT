/*
 * telecommands.h
 *
 *  Created on: Dec 11, 2021
 *      Author: psimo
 */

#ifndef INC_TELECOMMANDS_H_
#define INC_TELECOMMANDS_H_

#include "definitions.h"
#include "flash.h"

#define CONFIG_SIZE		13

void process_telecommand(uint8_t header, uint8_t info);

#endif /* INC_TELECOMMANDS_H_ */
