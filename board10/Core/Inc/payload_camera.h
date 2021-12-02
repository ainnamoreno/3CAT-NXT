/*
 * payload_camera.h
 *
 *  Created on: Nov 25, 2021
 *      Author: USUARIO
 */

#ifndef INC_PAYLOAD_CAMERA_H_
#define INC_PAYLOAD_CAMERA_H_

#include "stm32f4xx_hal.h"
//#include "main.h"
#include <string.h> // Usado para la funcion memcmp
#include <stdio.h>
#include <stdbool.h> //PARA EL BOOL

#include <stdlib.h> //PARA GUARDAR DATOS



uint8_t readResponse(UART_HandleTypeDef *huart, uint8_t expLength, uint8_t attempts);
bool runCommand(UART_HandleTypeDef *huart, uint8_t command, uint8_t *hexData, uint8_t dataArrayLength, uint8_t expLength, bool doFlush);
void getFrameLength(UART_HandleTypeDef *huart);
void retrieveImage(UART_HandleTypeDef *huart);


int min(int bSize, int frameLength);


#endif /* INC_PAYLOAD_CAMERA_H_ */
