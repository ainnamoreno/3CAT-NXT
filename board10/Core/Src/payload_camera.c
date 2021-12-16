//VERSION FUNCIONAL
/*
 * payload_camera.c
 *
 *  Created on: Nov 25, 2021
 *      Author: USUARIO
 */

#include <payload_camera.h>
#include <flash.h>

//VARIABLES
uint8_t dataBuffer[256], bufferLength; // TO DO: longitud del array dataBuffer = bSize! No se porque no puedo ponerlo directamente. No usar pow, usar <<=
uint32_t frameLength;
uint16_t framePointer;

uint8_t commInit[2] = {0x56, 0x00};
uint8_t commCapture = 0x36;
uint8_t bSize = 128; //TO DO: No guardar la longitud del buffer sino la potencia de la longitud del buffer ya que siempre debe tener base 2

//NO ACABADA
uint8_t readResponse(UART_HandleTypeDef *huart, uint8_t expLength, uint8_t attempts){
//  int i = 0;
//  bufferLength = 0;
//TO DO: Comprobar ack correctamente
//  while (attempts != i && bufferLength != expLength)//si attemps == i o si bufferLenght == expLength sale del while
//  {
////    if ()//mirar si tenemos algo que leer
////    { // Is there any data?
////      delay(1);
////      i++;
////      continue;
////    }
//    i = 0;
//    bufferLength++;
//    //dataBuffer[bufferLength++] = swsInstance.read(); // And we fill it with data
//	//for(int i = 0; i<)
//    HAL_UART_Receive(huart, dataBuffer, expLength, 100);
//  }

  HAL_UART_Receive(huart, dataBuffer, expLength, 100);

  return expLength;
}

bool runCommand(UART_HandleTypeDef *huart, uint8_t command, uint8_t *hexData, uint8_t dataArrayLength, uint8_t expLength, bool doFlush)
{ // Flushes the buffer, sends the command and hexData then checks and verifies it

//TO DO: buscar manera de vaciar el buffer de entrada
// Flush the reciever buffer
  if (doFlush)
  {
    readResponse(huart, 100, 10);
  }

  // Send the data
  HAL_UART_Transmit(huart, commInit, 2, HAL_MAX_DELAY);
  HAL_UART_Transmit(huart, &command, 1, HAL_MAX_DELAY);

  for (int i = 0; i < dataArrayLength; i++)
  {
	  HAL_UART_Transmit(huart, &hexData[i], 1, HAL_MAX_DELAY);
  }

  // Check the data
  if (readResponse(huart, expLength, 100) != expLength)
  {
    return false;
  }

  // Data should always be 76, 00, command, 00
  return dataBuffer[0] == 0x76 &&
	  dataBuffer[1] == 0x0 &&
	  dataBuffer[2] == command &&
	  dataBuffer[3] == 0x0;
//FUNCIONA
//  if (dataBuffer[0] != 0x76 ||
//      dataBuffer[1] != 0x0 ||
//      dataBuffer[2] != command ||
//      dataBuffer[3] != 0x0)
//  {
//    return false;
//  }
//
//  return true;
//HASTA AQUI
}

void getFrameLength(UART_HandleTypeDef *huart)
{ // ~ Get frame length
  uint8_t hexData[] = {0x01, 0x00};
  if (!runCommand(huart, 0x34, hexData, sizeof(hexData), 9, true))
  {
	  //TO DO: Hacer protocolo en caso de error (runCommand = false)
	  HAL_Delay(1);
  }
  frameLength = dataBuffer[5]; //Recreating split hex numbers from 4 bytes
  frameLength <<= 8;
  frameLength |= dataBuffer[6];
  frameLength <<= 8;
  frameLength |= dataBuffer[7];
  frameLength <<= 8;
  frameLength |= dataBuffer[8];
}

void retrieveImage(UART_HandleTypeDef *huart)
{ // * Retrieve photo data
	uint8_t dataVect[frameLength];

	//TO DO: utilizar memset y no bucle
	for(int i = 0; i < frameLength; i++){
	  dataVect[i] = 0;
	}

	framePointer = 0;

	while (frameLength > 0)
	{
		HAL_Delay(100);

		int toRead = min(bSize, frameLength); // Bytes read each loop
		uint8_t hexData[] = {0x0C, 0x0, 0x0A, 0x0, 0x0,
							 framePointer >> 8, framePointer & 0xFF, 0x0, 0x0,
							 0x0, toRead, 0x0, 0x0A
							};

		if (!runCommand(huart, 0x32, hexData, sizeof(hexData), 5, false))
		{
			HAL_Delay(1);
		}
		if (readResponse(huart, toRead + 5, 0xff) == 0) // +5 for verification header
		{
			HAL_Delay(1);
		}

		//AIXO HO HAURIEM DE MODIFICAT AMB OBC
		for(int i = 0; i < toRead; i++){
		  dataVect[framePointer+i] = dataBuffer[i];
		}

		framePointer += toRead;
		frameLength -= toRead;
	}

	Flash_Write_Data(PHOTO_ADDR, dataVect, sizeof(dataVect));
}

int min(int x, int y)
{
  return (x < y) ? x : y;
}


//VERSION BETA
///*
// * payload_camera.c
// *
// *  Created on: Nov 25, 2021
// *      Author: USUARIO
// */
//
//#include <payload_camera.h>
//#include <flash.h>
//
//
////VARIABLES
//
//uint8_t bSize = 128; //TO DO: No guardar la longitud del buffer sino la potencia del buffer ya que siempre debe tener base 2
//uint8_t dataBuffer[128], bufferLength; // TO DO: longitud del array dataBuffer = bSize! No se porque no puedo ponerlo directamente. No usar pow, usar <<=
//uint32_t frameLength;
//uint16_t framePointer;
//
//uint8_t commInit[2] = {0x56, 0x00};
//uint8_t commCapture = 0x36;
//
////COMANDS
//uint8_t captureImage[] = {0x36, 0x01, 0x00};
//uint8_t readDataLength[] = {0x34, 0x01, 0x00};
////uint8_t readImageData[] = {0x32, 0x0C, 0x0, 0x0A, 0x0, 0x0,
////					0x0, 0x0, 0x0, 0x0,
////					0x0, 0x0, 0x0, 0x0A
////					};
//
//
//
//
////NO ACABADA
//uint8_t readResponse(UART_HandleTypeDef *huart, uint8_t expLength, uint8_t attempts)
//{// Places response in the dataBuffer and returns its length
//  int i = 0;
//  bufferLength = 0;
//
////  while (attempts != i && bufferLength != expLength)//si attemps == i o si bufferLenght == expLength sale del while
////  {
//////    if ()//mirar si tenemos algo que leer
//////    { // Is there any data?
//////      delay(1);
//////      i++;
//////      continue;
//////    }
////    i = 0;
////    bufferLength++;
////    //dataBuffer[bufferLength++] = swsInstance.read(); // And we fill it with data
////	//for(int i = 0; i<)
////    HAL_UART_Receive(huart, dataBuffer, expLength, 100);
////  }
//  HAL_UART_Receive(huart, dataBuffer, expLength, 100);
//
//  return expLength;
//}
//
//bool runCommand(UART_HandleTypeDef *huart, uint8_t *command, uint8_t commandLength, uint8_t expLength, bool doFlush)
//{ // Flushes the buffer, sends the command then checks and verifies it
//
//// Flush the reciever buffer
////TO DO: buscar manera de vaciar el buffer de entrada
//  if (doFlush)
//  {
//    readResponse(huart, 100, 10);
//  }
//
//  // Send the data
//  HAL_UART_Transmit(huart, commInit, 2, HAL_MAX_DELAY);
//  HAL_UART_Transmit(huart, command, commandLength, HAL_MAX_DELAY);
//
//
//  // Check the data length
//  if (readResponse(huart, expLength, 100) != expLength)
//  {
//    return false;
//  }
//
//  // Data ack should always be 76, 00, command[0], 00
//  return dataBuffer[0] == 0x76 &&
//	  dataBuffer[1] == 0x0 &&
//	  dataBuffer[2] == command[0] &&
//	  dataBuffer[3] == 0x0;
//}
//
//bool getFrameLength(UART_HandleTypeDef *huart)
//{ // Get frame length
//  if (!runCommand(huart, readDataLength, sizeof(readDataLength), 9, true))
//  {
//		//TO DO: Hacer protocolo en caso de error (runCommand = false)
//		HAL_Delay(1);
//		return false;
//  }
//
//  //Recreating split hex numbers from 4 bytes
//  frameLength |= dataBuffer[7];
//  frameLength <<= 8;
//  frameLength |= dataBuffer[8];
//
//  return true;
//}
//
//void retrieveImage(UART_HandleTypeDef *huart)
//{ // * Retrieve photo data
//	uint8_t dataVect[frameLength];
//
//	//TO DO: utilizar memset y no bucle
//	for(int i = 0; i < frameLength; i++){
//	  dataVect[i] = 0;
//	}
//
//	while (frameLength > 0)
//	{
//		HAL_Delay(100);
//
//		int toRead = min(bSize, frameLength); // Bytes read each loop
////		uint8_t hexData[] = {0x0C, 0x0, 0x0A, 0x0, 0x0,
////							 framePointer >> 8, framePointer & 0xFF, 0x0, 0x0,
////							 0x0, toRead, 0x0, 0x0A
////							};
//		uint8_t readImageData[] = {0x00, 0x0C, 0x0, 0x0A, 0x0, 0x0,
//							 framePointer >> 8, framePointer & 0xFF, 0x0, 0x0,
//							 0x0, toRead, 0x0, 0x0A
//							};
////		readImageData[6] = framePointer >> 8;
////		readImageData[7] = framePointer & 0xFF;
////		readImageData[11] = toRead;
//
//
//		if (!runCommand(huart, readImageData, sizeof(readImageData), 5, false))
//		{
//
//			HAL_Delay(1);
//		}
//		if (readResponse(huart, toRead + 5, 0xff) == 0) // +5 for verification header
//		{
//			HAL_Delay(1);
//		}
//
//		for(int i = 0; i < toRead; i++){
//			dataVect[framePointer+i] = dataBuffer[i];
//		}
//
//		framePointer += toRead;
//		frameLength -= toRead;
//	}
//
//	//Flash_Write_Data(PHOTO_ADDR, dataVect, sizeof(dataVect));
//}
//
//int min(int x, int y)
//{
//  return (x < y) ? x : y;
//}
//
//void captureAndRetrieveImage(UART_HandleTypeDef *huart)
//{
//	//Captures Image
//	if (!runCommand(huart, captureImage, sizeof(captureImage), 5, true))
//	{
//		//TO DO: Hacer protocolo en caso de error (runCommand = false)
//		HAL_Delay(1);
//	}
//
//	if (!getFrameLength(huart))
//	{
//		//TO DO: Hacer protocolo en caso de error (getFrameLength = false)
//		HAL_Delay(1);
//	}
//
//	retrieveImage(huart);
//	//Gets frameLength
//
//
//}
