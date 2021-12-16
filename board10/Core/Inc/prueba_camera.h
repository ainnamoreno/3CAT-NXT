/*
 * payload_camera.h
 *
 *  Created on: Nov 25, 2021
 *      Author: USUARIO
 */

#ifndef INC_PAYLOAD_CAMERA_H_
#define INC_PAYLOAD_CAMERA_H_

//INCLUDES
#include "stm32f4xx_hal.h"
//#include "main.h"
#include <string.h> // Usado para la funcion memcmp
#include <stdio.h>
#include <stdbool.h> //PARA EL BOOL
#include <stdlib.h> //PARA GUARDAR DATOS

//VARIABELS
//JM Camera Variables
//uint8_t reset[4] = {0x56, 0x00, 0x26, 0x00}; // dec{86, 0, 38, 0};
//uint8_t resetAck[4] = {0x76, 0x00, 0x26, 0x00}; //JM: CREO QUE DEBERIA AÑADIRSE UN 0x00
//
//uint8_t captureImage[5] = {0x56, 0x00, 0x36, 0x01, 0x00};
//uint8_t captureImageACK[5] = {0x76, 0x00, 0x36, 0x00, 0x00};
//
//uint8_t readImageDataLength[5] = {0x56, 0x00, 0x34, 0x01, 0x00};//dec{86, 0, 52, 1, 0}
//uint8_t readImageDataLengthAck[7] = {0x76, 0x00, 0x34, 0x00, 0x04, 0x00, 0x00};//dec{118, 0, 52, 0, 4, 0, 0}
//
//uint16_t imageLength;
//
//uint8_t readImageData[16] = {0x56, 0x00, 0x32, 0x0C, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A};
//uint8_t readImageDataAck[5] = {0x76, 0x00, 0x32, 0x00};
//
//uint8_t stopCapture[5] = {0x56, 0x00, 0x36, 0x01, 0x03};// dec{86,0,54,1,3}
//uint8_t stopCaptureAck[5] = {0x76, 0x00, 0x36, 0x00, 0x00};// dec{}
//
//uint8_t setCompressibility[9] = {0x56, 0x00, 0x31, 0x05, 0x01, 0x01, 0x12, 0x04, 0xFF};
//uint8_t setBandRate[10] = {0x56, 0x00, 0x31, 0x06, 0x04, 0x02, 0x00, 0x08, 0x0D, 0xA6};
//
//uint8_t dataBuffer[200+1];
//uint8_t bSize = 128;

uint8_t readResponse(UART_HandleTypeDef *huart, uint8_t expLength, uint8_t attempts);
bool runCommand(UART_HandleTypeDef *huart, uint8_t command, uint8_t *hexData, uint8_t dataArrayLength, uint8_t expLength, bool doFlush);
void getFrameLength(UART_HandleTypeDef *huart);
void retrieveImage(UART_HandleTypeDef *huart);


int min(int bSize, int frameLength);


#endif /* INC_PAYLOAD_CAMERA_H_ */
