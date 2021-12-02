/*
 * sensorReadings.h
 *
 *  Created on: Apr 28, 2021
 *      Author: guifre
 */

#ifndef INC_SENSORREADINGS_H_
#define INC_SENSORREADINGS_H_

//#include "main.h"
#include "definitions.h"

/*Overwrites into the Temperatures Union the temperature of the 8 sensors*/
void acquireTemp(I2C_HandleTypeDef *hi2c);

/*Overwrites into the Voltages Union the readings of the 12 voltages*/
void acquireVoltage();

/*Overwrites into the Currents Union the readings of the 7 currents*/
void acquireCurrents();

/*Includes the functions above*/
void sensorReadings(I2C_HandleTypeDef *hi2c);

#endif /* INC_SENSORREADINGS_H_ */
