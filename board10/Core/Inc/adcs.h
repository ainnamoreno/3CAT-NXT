/*!
 * \file      adcs.h
 *
 * \brief     ADCS subsystem funcitons
 *
 *
 * \created on: 14/12/2021
 *
 * \author    Pol Simon
 *
 * \author    David Reiss
 */

#ifndef INC_ADCS_H_
#define INC_ADCS_H_

#include "stm32f4xx_hal.h"

typedef struct __attribute__ ((__packed__)) gyro_aux {
	double gx_h;
	double gy_h;
	double gz_h;
}gyro_aux;

/*Detumble the satellite (ADCS subsystem)
 *Once it is stabilized, write detumble_state = true in the EEPROM memory */
void detumble(I2C_HandleTypeDef *hi2c);

void tumble(I2C_HandleTypeDef *hi2c);

double AngularVelocity(I2C_HandleTypeDef *hi2c1, double *w);

void readPhotodiodes(ADC_HandleTypeDef *hadc);

void singlePhotodiode(ADC_HandleTypeDef *hadc);

double MagneticField(I2C_HandleTypeDef *hi2c1, double *m);

double cross(double *A, double *B, double *res);

double norm(double A[]);

void CurrentToCoil(double intensidad[3]);


#endif /* INC_ADCS_H_ */
