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
#include "math.h"

double k=0;

/**************************************************************************************
 *                                                                                    *
 * Function:  cross                                                 		  		  *
 * --------------------                                                               *
 * Calculates the cross product between two vectors					  				  *
 *															                          *
 *  returns: vector with the cross product									          *
 *                                                                                    *
 **************************************************************************************/
void* cross(double A[3], double B[3])
 {
	double x, y, z;
	x = A[1]*B[2]-A[2]*B[1];
	y = A[2]*B[0]-A[0]*B[2];
	z = A[0]*B[1]-A[1]*B[0];
	double q[3] = {x, y, z};
	return q;

 }
/**************************************************************************************
 *                                                                                    *
 * Function:  norm                                                 		  		      *
 * --------------------                                                               *
 * Calculates the norm of a vectors					  				                  *
 *															                          *
 *  returns: integer with the value of the norm of the vector						  *
 *                                                                                    *
 **************************************************************************************/
double norm(double A[3]){

	double vect_norm=0;
	vect_norm = sqrt((A[0]+A[1]+A[2]));
	return vect_norm;

}
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

	double IdealTorque, MagneticDipole;
	double w_target[];
	double mag_read[];
	w_target= AngularVelocity();
	mag_read= MagneticField();
	IdealTorque = -(k*w_target);
	MagneticDipole = cross(mag_read,IdealTorque)/pow(norm(mag_read),2);




}
/**************************************************************************************
 *                                                                                    *
 * Function:  tumble                                                 		  		  *
 * --------------------                                                               *
 * Checks the gyroscope measurements and destabilizes the satellite. 					  *
 * It is called when the satellite is experiencing a lot of heat and wants to cool down					  *
 *                                                                                    *
 *  hi2c: I2C to read outputs from gyroscope					    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void tumble(I2C_HandleTypeDef *hi2c) {

	double IdealTorque, MagneticDipole;
	double w_target[];
	double mag_read[];
	w_target= AngularVelocity();
	mag_read= MagneticField();
	IdealTorque = (k*w_target);
	MagneticDipole = cross(mag_read,IdealTorque)/pow(norm(mag_read),2);



}
/****************************************************************************/
/* AngularVelocity: This function read the data from the gyroscope */
/****************************************************************************/
void* AngularVelocity(I2C_HandleTypeDef *hi2c1){
	uint8_t gx[2];
	uint8_t gy[2];
	uint8_t gz[2];
	double s = 131;
	gyro_aux gyrox;
	HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, 0x43, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x68<<1, gx, 2, 1000);
	gyrox.gx_h = (double)((uint8_t)gx[0]<<8|gx[1]); //Comprobar que els valors tenen sentit (la conversió)
	HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, 0x45, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x68<<1, gy, 2, 1000);
	gyrox.gy_h = (double)((uint8_t)gy[0]<<8|gy[1]);
	HAL_I2C_Master_Transmit(&hi2c1, 0x68<<1, 0x47, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x68<<1, gz, 2, 1000);
	gyrox.gz_h = (double)((uint8_t)gz[0]<<8|gz[1]);
	//After we have readed the data from the gyro we have to divide by 131
	//Check the datasheet for more information
	double w[] = {gyrox.gx_h/s, gyrox.gy_h/s, gyrox.gz_h/s};
	return w;
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
void readPhotodiodes(ADC_HandleTypeDef *hadc) { // I think the four ADC should be passed as parameters
	/*3 photodiodes are directly connected to 3 of the 4 ADC pins
	 * the other 3 photodiodes are connected through the inputs 1,2 and 3 of a multiplexor*/
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

/****************************************************************************/
/* MagneticField: This function read the data from the magnetorquer */
/****************************************************************************/
void* MagneticField(I2C_HandleTypeDef *hi2c1){
	uint8_t x[2], y[2], z[2];
	double mx, my, mz;
	//Check the datasheet to see how the data comes
	HAL_I2C_Master_Transmit(&hi2c1, 0x30<<1, 0x00, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x30<<1, x[0], 1, 1000);
	HAL_I2C_Master_Transmit(&hi2c1, 0x30<<1, 0x01, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x30<<1, x[1], 1, 1000);
	HAL_I2C_Master_Transmit(&hi2c1, 0x30<<1, 0x02, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x30<<1, y[0], 1, 1000);
	HAL_I2C_Master_Transmit(&hi2c1, 0x30<<1, 0x03, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x30<<1, y[1], 1, 1000);
	HAL_I2C_Master_Transmit(&hi2c1, 0x30<<1, 0x04, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x30<<1, z[0], 1, 1000);
	HAL_I2C_Master_Transmit(&hi2c1, 0x30<<1, 0x05, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x30<<1, z[1], 1, 1000);
	mx = (double)((uint8_t)x[0]<<8|x[1]);
	my = (double)((uint8_t)y[0]<<8|y[1]);
	mz = (double)((uint8_t)z[0]<<8|z[1]);
	double m[3] = {mx, my, mz};
	return m;
}


