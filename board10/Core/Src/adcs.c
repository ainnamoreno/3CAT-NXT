/*!
 * \file      adcs.c
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

#include "adcs.h"

/**************************************************************************************
 *                                                                                    *
 * Function:  detumble                                                 		  		  *
 * --------------------                                                               *
 * Checks the gyroscope measurements and stabilizes the satellite. 					  *
 * It is called when the satellite is ejected from the deployer						  *
 *                                                                                    *
 *  hi2c: I2C to read outputs from gyroscope					    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void detumble(I2C_HandleTypeDef *hi2c) {

}

/**************************************************************************************
 *                                                                                    *
 * Function:  readPhotodiodes                                                 		  *
 * --------------------                                                               *
 * Obtains the output values from all the photodiodes, varying the selectors		  *
 * of the multiplexor (GPIOs PA11 and PA12) 										  *
 *                                                                                    *
 *  hadc: ADC to read outputs from the photodiodes				    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void readPhotodiodes(ADC_HandleTypeDef *hadc) { // crec que s'haurien de passar els 4 ADC
	/*3 photodiodes estan directament connectats a 3 dels 4 ADCs
	 * els altres 3 photodiodes estan a les entrades 1,2 i 3 d'un multiplexor*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	singlePhotodiode(hadc);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	singlePhotodiode(hadc);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	singlePhotodiode(hadc);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

}

/**************************************************************************************
 *                                                                                    *
 * Function:  singlePhotodiode                                         		  		  *
 * --------------------                                                               *
 * Obtains the output value from a single photodiode.								  *
 *                                                                                    *
 *  hadc: ADC to read outputs from the photodiodes				    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void singlePhotodiode(ADC_HandleTypeDef *hadc) {

}
