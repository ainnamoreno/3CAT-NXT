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
#include "stdbool.h"
#include "sgp.h"
#include "stdlib.h"

#define PI 3.14159265359



void matrix_prod3x3(Matrix3x3 *m1, Matrix3x3 *m2, Matrix3x3 *res)
{
    res->col1[0] = m1->col1[0]*m2->col1[0]+m1->col2[0]*m2->col1[1]+m1->col3[0]*m2->col1[2];
    res->col1[1] = m1->col1[1]*m2->col1[0]+m1->col2[1]*m2->col1[1]+m1->col3[1]*m2->col1[2];
    res->col1[2] = m1->col1[2]*m2->col1[0]+m1->col2[2]*m2->col1[1]+m1->col3[2]*m2->col1[2];

    res->col2[0] = m1->col1[0]*m2->col2[0]+m1->col2[0]*m2->col2[1]+m1->col3[0]*m2->col2[2];
    res->col2[1] = m1->col1[1]*m2->col2[0]+m1->col2[1]*m2->col2[1]+m1->col3[1]*m2->col2[2];
    res->col2[2] = m1->col1[2]*m2->col2[0]+m1->col2[2]*m2->col2[1]+m1->col3[2]*m2->col2[2];

    res->col3[0] = m1->col1[0]*m2->col3[0]+m1->col2[0]*m2->col3[1]+m1->col3[0]*m2->col3[2];
    res->col3[1] = m1->col1[1]*m2->col3[0]+m1->col2[1]*m2->col3[1]+m1->col3[1]*m2->col3[2];
    res->col3[2] = m1->col1[2]*m2->col3[0]+m1->col2[2]*m2->col3[1]+m1->col3[2]*m2->col3[2];
}

/**************************************************************************************
 *                                                                                    *
 * Function:  cross                                                 		  		  *
 * --------------------                                                               *
 * Calculates the cross product between two vectors					  				  *
 *															                          *
 *  returns: vector with the cross product									          *
 *                                                                                    *
 **************************************************************************************/
void cross(double *A, double *B, double *res){

	res[0] = A[1]*B[2]-A[2]*B[1];
	res[1] = A[2]*B[0]-A[0]*B[2];
	res[2] = A[0]*B[1]-A[1]*B[0];

 }

void decimal_to_binary(int n, char *res)
{
  int c, d, t;
  char *p;

  t = 0;
  p = (char*)malloc(32+1);

  for (c = 8 ; c > 0 ; c--)
  {
	//  dividing n with 2^c
    d = n >> c;

    if (d & 1)
      *(p+t) = 1 + '0';
    else
      *(p+t) = 0 + '0';

    t++;
  }
  *(p+t) = '\0';

  res = p;
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

double gainConstant(void){

	double w = 0;
	double T = 5551;
	double m = 0.250;
	double a = 0.05;
	double alfa = 0.645772;
	double k = 0;
	double inercia [3][3] = {{(m*a*a)/6, 0, 0}, {0, (m*a*a)/6, 0}, {0, 0, (m*a*a)/6}};
	w = (2*PI)/T;
	k = 2*w*(1+sin(alfa))*inercia[0][0];

	return k;

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
void detumble(I2C_HandleTypeDef *hi2c1) {

	double gyr_read[3], mag_read[3], intensity[3], magneticDipole[3], w[3], m[3], a;
	double maxMagneticDipole1 = 0.02359;
	double maxMagneticDipole2 = 0.03234;
	double maxIntensity1 = 0.14681;
	double maxIntensity2 = 0.1495;
	AngularVelocity(hi2c1, w);
	MagneticField(hi2c1,m);
	memcpy(gyr_read, w, 3);
	memcpy(mag_read, m, 3);
	cross(mag_read,gyr_read, magneticDipole);
	a = pow(norm(magneticDipole),2);
	double k = gainConstant();
	magneticDipole[0] = -k*magneticDipole[0]/a;
	magneticDipole[1] = -k*magneticDipole[1]/a;
	magneticDipole[2] = -k*magneticDipole[2]/a;

	if(magneticDipole[0]>0){
		intensity[0] = (magneticDipole[0]*maxIntensity1)/maxMagneticDipole1;
	}else{
		intensity[0] = (magneticDipole[0]*maxIntensity2)/maxMagneticDipole2;
	}

	intensity[1] = (magneticDipole[1]*maxIntensity2)/maxMagneticDipole2;
	intensity[2] = (magneticDipole[2]*maxIntensity2)/maxMagneticDipole2;
	CurrentToCoil(hi2c1, intensity);


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
void tumble(I2C_HandleTypeDef *hi2c1) {

	double k=0;
	double gyr_read[3], mag_read[3], intensity[3], magneticDipole[3], w[3], m[3], a;
	double maxMagneticDipole1 = 0.02359;
	double maxMagneticDipole2 = 0.03234;
	double maxIntensity1 = 0.14681;
	double maxIntensity2 = 0.1495;
	AngularVelocity(hi2c1, w);
	MagneticField(hi2c1,m);
	memcpy(gyr_read, w, 3);
	memcpy(mag_read, m, 3);

	cross(mag_read,gyr_read, magneticDipole);
	a = pow(norm(magneticDipole),2);
	magneticDipole[0] = k*magneticDipole[0]/a;
	magneticDipole[1] = k*magneticDipole[1]/a;
	magneticDipole[2] = k*magneticDipole[2]/a;

	if(magneticDipole[0]>0){
		intensity[0] = (magneticDipole[0]*maxIntensity1)/maxMagneticDipole1;
	}else{
		intensity[0] = (magneticDipole[0]*maxIntensity2)/maxMagneticDipole2;
	}

	intensity[1] = (magneticDipole[1]*maxIntensity2)/maxMagneticDipole2;
	intensity[2] = (magneticDipole[2]*maxIntensity2)/maxMagneticDipole2;
	CurrentToCoil(hi2c1, intensity);


}
/****************************************************************************/
/* AngularVelocity: This function read the data from the gyroscope 			*/
/****************************************************************************/
void AngularVelocity(I2C_HandleTypeDef *hi2c1, double *w){
	uint8_t gx[2];
	uint8_t gy[2];
	uint8_t gz[2];
	double s = 131;
	gyro_aux gyrox;
	HAL_I2C_Master_Transmit(hi2c1, 0x68<<1, 0x43, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x68<<1, gx, 2, 1000);
	gyrox.gx_h = (double)((uint8_t)gx[0]<<8|gx[1]); //Comprobar que els valors tenen sentit (la conversió)
	HAL_I2C_Master_Transmit(hi2c1, 0x68<<1, 0x45, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x68<<1, gy, 2, 1000);
	gyrox.gy_h = (double)((uint8_t)gy[0]<<8|gy[1]);
	HAL_I2C_Master_Transmit(hi2c1, 0x68<<1, 0x47, 1, 1000);
	HAL_I2C_Master_Receive(hi2c1, 0x68<<1, gz, 2, 1000);
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
	singlePhotodiode(hadc);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	singlePhotodiode(hadc);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	singlePhotodiode(hadc);
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

bool CheckGyro(I2C_HandleTypeDef *hi2c1){

	double gyr_read[3], v[3];
	AngularVelocity(hi2c1, v);
	memcpy(gyr_read, v, 3);
	if( gyr_read[0]<=0.5 && gyr_read[1]<=0.5 && gyr_read[2]<=0.5 ){
		return true;
	}
	return false;
}

/****************************************************************************/
/* MagneticField: This function read the data from the magnetometer */
/****************************************************************************/
void MagneticField(I2C_HandleTypeDef *hi2c1, double *m){
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
void CurrentToCoil(I2C_HandleTypeDef *hi2c1, double intensidad[3]){

	int auxCurrent[3];
	char *currentToApply, *data_1, *data_2, *signCurrent;
	char finalCurrent[16] = {1,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0};
	auxCurrent[0]=round(intensidad[0]*1023/0.150);
	auxCurrent[1]=round(intensidad[1]*1023/0.150);
	auxCurrent[2]=round(intensidad[2]*1023/0.150);
	uint8_t data[4] = {0x66, 0x00, 0x00, 0x90};

	if(auxCurrent[0]>0){//LV1

		decimal_to_binary(auxCurrent[0], currentToApply);
		for(int i=0; i<10;i++){
			finalCurrent[i+2] = currentToApply[i];
			data_1[i] = finalCurrent[i];

		}
		for(int j=0; j<4; j++){
			data_2[j] = finalCurrent[j+8];
		}
		data[1] = (uint8_t)*data_1;
		data[2] = (uint8_t)*data_2;
		HAL_I2C_Master_Transmit_DMA(hi2c1, 0x70<<1, data, 4);


	}else{//LV6

		signCurrent[0] = sign(auxCurrent[0])*auxCurrent[0];
		decimal_to_binary(signCurrent[0], currentToApply);
		for(int i=0; i<10;i++){
			finalCurrent[i+2] = currentToApply[i];
			data_1[i] = finalCurrent[i];

		}
		for(int j=0; j<4; j++){
			data_2[j] = finalCurrent[j+8];
		}
		data[1] = (uint8_t)*data_1;
		data[2] = (uint8_t)*data_2;
		HAL_I2C_Master_Transmit_DMA(hi2c1, 0x75<<1, data, 4);

	}
	if(auxCurrent[1]>0){//LV2

		decimal_to_binary(auxCurrent[1], currentToApply);
		for(int i=0; i<10;i++){
			finalCurrent[i+2] = currentToApply[i];
			data_1[i] = finalCurrent[i];

		}
		for(int j=0; j<4; j++){
			data_2[j] = finalCurrent[j+8];
		}
		data[1] = (uint8_t)*data_1;
		data[2] = (uint8_t)*data_2;

		HAL_I2C_Master_Transmit_DMA(hi2c1, 0x71<<1, data, 4);

	}else{//LV3

		signCurrent[1] = sign(auxCurrent[1])*auxCurrent[1];
		decimal_to_binary(signCurrent[1], currentToApply);
		for(int i=0; i<10;i++){
			finalCurrent[i+2] = currentToApply[i];
			data_1[i] = finalCurrent[i];

		}
		for(int j=0; j<4; j++){
			data_2[j] = finalCurrent[j+8];
		}
		data[1] = (uint8_t)*data_1;
		data[2] = (uint8_t)*data_2;
		HAL_I2C_Master_Transmit_DMA(hi2c1, 0x72<<1, data, 4);

	}
	if(auxCurrent[2]>0){//LV5

		decimal_to_binary(auxCurrent[2], currentToApply);
		for(int i=0; i<10;i++){
			finalCurrent[i+2] = currentToApply[i];
			data_1[i] = finalCurrent[i];

		}
		for(int j=0; j<4; j++){
			data_2[j] = finalCurrent[j+8];
		}
		data[1] = (uint8_t)*data_1;
		data[2] = (uint8_t)*data_2;
		HAL_I2C_Master_Transmit_DMA(hi2c1, 0x74<<1, data, 4);

	}else{//LV4

		signCurrent[2] = sign(auxCurrent[2])*auxCurrent[2];
		decimal_to_binary(signCurrent[1], currentToApply);
		for(int i=0; i<10;i++){
			finalCurrent[i+2] = currentToApply[i];
			data_1[i] = finalCurrent[i];

		}
		for(int j=0; j<4; j++){
			data_2[j] = finalCurrent[j+8];
		}
		data[1] = (uint8_t)*data_1;
		data[2] = (uint8_t)*data_2;
		HAL_I2C_Master_Transmit_DMA(hi2c1, 0x73<<1, data, 4);

	}




}


void nadir_algorithm(I2C_HandleTypeDef *hi2c1, float euler[3], float angle0[2]){


	double gyr_read[3], mag_read[3], intensity[3], magneticDipole[3], w[3], m[3], a;
	double kp[3] = {5.49359905270580e-06, 3.80118411237760e-06, 8.02831030719022e-14};
	double kd[3] = {0.000206390535670685, 0.000156635557926080, 5.39645941623300e-08};
	double maxMagneticDipole1 = 0.02359;
	double maxMagneticDipole2 = 0.03234;
	double maxIntensity1 = 0.14681;
	double maxIntensity2 = 0.1495;
	angle0[0] = 0;
	angle0[1] = 0;
	float angle_e[3];
	double idealTorque[3] = {0, 0, 0};

	angle_e[0] = &euler[3]-angle0;
	angle_e[1] = &euler[2]-angle0;
	angle_e[2] = 0;
	AngularVelocity(hi2c1, w);
	MagneticField(hi2c1,m);
	memcpy(gyr_read, w, 3);
	memcpy(mag_read, m, 3);

	idealTorque[0] = -(kp[0]*angle_e[0]+kd[0]*gyr_read[0]*180/PI);
	idealTorque[1] = -(kp[1]*angle_e[1]+kd[1]*gyr_read[1]*180/PI);
	idealTorque[2] = -(kp[2]*angle_e[2]+kd[2]*gyr_read[2]*180/PI);
	cross(mag_read,idealTorque, magneticDipole);
	a = pow(norm(magneticDipole),2);
	magneticDipole[0] = magneticDipole[0]/a;
	magneticDipole[1] = magneticDipole[1]/a;
	magneticDipole[2] = magneticDipole[2]/a;
	if(magneticDipole[0]>0){
		intensity[0] = (magneticDipole[0]*maxIntensity1)/maxMagneticDipole1;
	}else{
		intensity[0] = (magneticDipole[0]*maxIntensity2)/maxMagneticDipole2;
	}

	intensity[1] = (magneticDipole[1]*maxIntensity2)/maxMagneticDipole2;
	intensity[2] = (magneticDipole[2]*maxIntensity2)/maxMagneticDipole2;
	CurrentToCoil(hi2c1, intensity);

}





