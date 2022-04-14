/*
 * payload_camera.h
 *
 *  Created on: Nov 25, 2021
 *      Author: Jaume
 */

#ifndef INC_PAYLOAD_CAMERA_H_
#define INC_PAYLOAD_CAMERA_H_

#include "stm32f4xx_hal.h"
#include <string.h> // Usado para la funcion memcmp
#include <stdio.h>
#include <stdbool.h> //PARA EL BOOL
#include <stdlib.h> //PARA GUARDAR DATOS
//todo hacer criba de las librerias, estraer solo las funciones utiles.
#include "board.h"


/**
  * Waits for data to be received and stores the information to the global variable dataBuffer
  */
uint8_t readResponse(UART_HandleTypeDef *huart, uint8_t expLength, uint8_t attempts);


/**
  * Transmitts information using the USART protocol
  * reads the response using readResponse
  * Veryfy if the received data is the expected one
  */
bool runCommand(UART_HandleTypeDef *huart, uint8_t command, uint8_t *hexData, uint8_t dataArrayLength, uint8_t expLength, bool doFlush);

/**
  * #2
  * Takes photo and saves it in the camera memory
  */
bool captureImage(UART_HandleTypeDef *huart);

/**
  * #3
  * Actualize the value of the variable frameLength with the new length of the image
  */
void getFrameLength(UART_HandleTypeDef *huart);

/**
  * #4
  * Saves the image to the flash memory of the STM32
  */
void retrieveImage(UART_HandleTypeDef *huart);

/**
  * #5
  * Stops the capture
  */
void stopCapture(UART_HandleTypeDef *huart);

/**
  * #6
  * Sets the compressibility of the image
  */
void setCopressibility(UART_HandleTypeDef *huart, uint8_t compressibility);

/**
  * #7
  * Sets the resolution of the image
  */
void setResolution(UART_HandleTypeDef *huart, uint8_t resolution);

/**
  * #8
  * Executes the different commands needed to take a photo
  * captureImage
  * getFrameLength
  * retrieveImage
  * stopCapture
  */
bool takePhoto(UART_HandleTypeDef *huart);


int min(int bSize, int frameLength);


#endif /* INC_PAYLOAD_CAMERA_H_ */
