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
#include "string.h"

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
double cross(double *A, double *B, double *res){

	res[0] = A[1]*B[2]-A[2]*B[1];
	res[1] = A[2]*B[0]-A[0]*B[2];
	res[2] = A[0]*B[1]-A[1]*B[0];

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

	double idealTorque[3], gyr_read[3], mag_read[3], intensity[3], magneticDipole[3], w[3], m[3];
	double maxMagneticDipole1 = 0.02359;
	double maxMagneticDipole2 = 0.03234;
	double maxIntensity1 = 0.14681;
	double maxIntensity2 = 0.1495;
	AngularVelocity(hi2c, w);
	MagneticField(hi2c,m);
	memcpy(gyr_read, w, 3);
	memcpy(mag_read, m, 3);
	idealTorque[0]=-k*gyr_read[0];
	idealTorque[1]=-k*gyr_read[1];
	idealTorque[2]=-k*gyr_read[2];
	cross(mag_read,idealTorque, magneticDipole)/pow(norm(mag_read),2);
	if(magneticDipole>0){
		intensity[0] = (magneticDipole[0]*maxIntensity1)/maxMagneticDipole1;
	}else{
		intensity[0] = (magneticDipole[0]*maxIntensity2)/maxMagneticDipole2;
	}

	intensity[1] = (magneticDipole[1]*maxIntensity2)/maxMagneticDipole2;
	intensity[2] = (magneticDipole[2]*maxIntensity2)/maxMagneticDipole2;
	CurrentToCoil(intensity);


}
/**************************************************************************************
 *                                                                                    *
 * Function:  tumble                                                 		  		  *
 * --------------------                                                               *
 * Checks the gyroscope measurements and destabilizes the satellite. 				  *
 * It is called when the satellite is experiencing a lot of heat and wants to cool dow*
 *                                                                                    *
 *  hi2c: I2C to read outputs from gyroscope					    				  *
 *															                          *
 *  returns: Nothing									                              *
 *                                                                                    *
 **************************************************************************************/
void tumble(I2C_HandleTypeDef *hi2c) {

	double idealTorque[3], gyr_read[3], mag_read[3], intensity[3], magneticDipole[3], w[3], m[3];
	double maxMagneticDipole1 = 0.02359;
	double maxMagneticDipole2 = 0.03234;
	double maxIntensity1 = 0.14681;
	double maxIntensity2 = 0.1495;
	AngularVelocity(hi2c, w);
	MagneticField(hi2c,m);
	memcpy(gyr_read, w, 3);
	memcpy(mag_read, m, 3);
	idealTorque[0]=k*gyr_read[0];
	idealTorque[1]=k*gyr_read[1];
	idealTorque[2]=k*gyr_read[2];
	cross(mag_read,idealTorque, magneticDipole)/pow(norm(mag_read),2);
	if(magneticDipole>0){
		intensity[0] = (magneticDipole[0]*maxIntensity1)/maxMagneticDipole1;
	}else{
		intensity[0] = (magneticDipole[0]*maxIntensity2)/maxMagneticDipole2;
	}

	intensity[1] = (magneticDipole[1]*maxIntensity2)/maxMagneticDipole2;
	intensity[2] = (magneticDipole[2]*maxIntensity2)/maxMagneticDipole2;
	CurrentToCoil(intensity);


}
/****************************************************************************/
/* AngularVelocity: This function read the data from the gyroscope */
/****************************************************************************/
double AngularVelocity(I2C_HandleTypeDef *hi2c1, double *w){
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
	w[0] = gyrox.gx_h/s;
	w[1] = gyrox.gy_h/s;
	w[2] = gyrox.gz_h/s;

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
	singlePhotodiode(&hadc);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	singlePhotodiode(&hadc);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	singlePhotodiode(&hadc);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

}

void sun_vector(ADC_HandleTypeDef *hadc, double *s){


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
double MagneticField(I2C_HandleTypeDef *hi2c1, double *m){
	uint8_t x[2], y[2], z[2];
	double mx, my, mz;
	//Check the datasheet to see how the data comes
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x00, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, x[0], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x01, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, x[1], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x02, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, y[0], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x03, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, y[1], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x04, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, z[0], 1, 1000);
	HAL_I2C_Master_Transmit(hi2c1, 0x30<<1, 0x05, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x30<<1, z[1], 1, 1000);
	mx = (double)((uint8_t)x[0]<<8|x[1]);
	my = (double)((uint8_t)y[0]<<8|y[1]);
	mz = (double)((uint8_t)z[0]<<8|z[1]);
	m[0] = mx;
	m[1] = my;
	m[2] = mz;

}

/*************************************************************************************/
/* CurrentToCoil: We use the intensity that we have calculated in the Bdot
*/
/* and we send it to the differents coils. We need to distinguish every coil */
/* LV1=+x LV2=+y LV3=-y LV4=-z LV5=+z LV6=-x
*/
/*************************************************************************************/
void CurrentToCoil(double intensidad[3]){
	//We can't distinguish the differents drivers in the I2C bus
	//We have seen in the datasheet that their addresses aren't configurable
	//To solve this we have thought in a I2C multiplexor so we cant change in the
	//differents drivers


}

