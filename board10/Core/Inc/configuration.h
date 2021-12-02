/*
 * configuration.h
 *
 *  Created on: Apr 20, 2021
 *      Author: guifre
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

//#include "main.h"
#include "definitions.h"


/*Only at the beginning, includes the Antenna deployment, check batteries, configure payloads*/
/*Will be executed every time we reboot the system*/
void init(bool detumble_state, bool deployment_state, bool deploymentRF_state, I2C_HandleTypeDef *hi2c);

/*Initialize all the sensors*/
void initsensors(I2C_HandleTypeDef *hi2c);

/*Compute the level of battery and updates batterylevel*/
int checkbatteries(void);

/*Check if the memory have enough space to store a photo/spectrum*/
bool checkmemory(void);

/*Check the temperature of the 8 sensors, returns true if the temperature is between TEMP_MAX and TEMP_MIN*/
bool checktemperature(I2C_HandleTypeDef *hi2c);

/*Activate or deactivate the battery heater*/
void heater(int state);

/*Send a signal to a resistor to burn a wire and deploy COMMS Antenna
 *Check the switch to make sure the Antenna has been deployed properly
 *And write deployment_state = true in the EEPROM memory*/
void deployment(bool deployment_state, I2C_HandleTypeDef *hi2c);

/*Send a signal to a resistor to burn a wire and deploy PL2 Antenna
 *Once confirmed the proper deployment of the antenna,  write deploymentRF_state = true in the EEPROM memory*/
void deploymentRF(void);

/*Detumble the satellite (ADCS subsystem)
 *Once it is stabilized, write detumble_state = true in the EEPROM memory */
void detumble(void);

 /*（1)power on
  *（2)delay 2.5s
  *（3）reset command
  *（4）set image resolution command -> 640x480 default
  *（5）set image compressibility command
  * Need 115200bps band rate at UART configuration
  * After that go to 'INIT STATE'*/
bool payloadconfig(UART_HandleTypeDef *huart);

/*We'll have a file (SPG4) which will tell us the position
 *Check if we are in the region of contact with GS
 *Also check if we are in the position of using payloads */
bool check_position(void);

/*Check battery level, temperatures,etc
 *If each parameter is between a specified values returns true*/
bool system_state(I2C_HandleTypeDef *hi2c);

//void writeFlash(TypeProgram, Adress, Data); //todo revisar

#endif /* INC_CONFIGURATION_H_ */
