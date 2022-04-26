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
#include "stdbool.h"

typedef struct __attribute__ ((__packed__)) gyro_aux {
	double gx_h;
	double gy_h;
	double gz_h;
}gyro_aux;

typedef struct __attribute__ ((__packed__)) gyro_data {
	double gx;
	double gy;
	double gz;
}gyro_data;

typedef struct __attribute__ ((__packed__)) mag_data {
	double mx;
	double my;
	double mz;
}mag_data;

typedef struct __attribute__ ((__packed__)) sun_vector {
	uint32_t x1;
	uint32_t x2;
	uint32_t y1;
	uint32_t y2;
	uint32_t z1;
	uint32_t z2;
}sun_vector;

typedef struct __attribute__ ((__packed__)) CalibSensorsData {
	double gx_h;
	double gy_h;
	double gz_h;
}CalibSensorsData;

typedef struct __attribute__ ((__packed__)) SensorsData {
	double gx_h;
	double gy_h;
	double gz_h;
}SensorsData;

typedef struct Matrix3x3 {
    float col1[3];
    float col2[3];
    float col3[3];
}Matrix3x3;

/*Detumble the satellite (ADCS subsystem)
 *Once it is stabilized, write detumble_state = true in the EEPROM memory */
void detumble(I2C_HandleTypeDef *hi2c);

void tumble(I2C_HandleTypeDef *hi2c);

void AngularVelocity(I2C_HandleTypeDef *hi2c1, double *w);

void readPhotodiodes(ADC_HandleTypeDef *hadc, uint32_t photoData[6]);

void MagneticField(I2C_HandleTypeDef *hi2c1, double *m);

void cross(double *A, double *B, double *res);

double norm(double A[]);

void CurrentToCoil(I2C_HandleTypeDef *hi2c1, double intensidad[3]);

bool CheckGyro();

void decimal_to_binary(int n, char *res);

void nadir_algorithm(I2C_HandleTypeDef *hi2c1, float euler[3], float q_est[4]);

void matrix_prod3x3(Matrix3x3 *m1, Matrix3x3 *m2, Matrix3x3 *res);

double gainConstant(void);

void sensorData(I2C_HandleTypeDef *hi2c1, ADC_HandleTypeDef *hadc, mag_data *magData, gyro_data *gyroData, sun_vector *sunVector);


#endif /* INC_ADCS_H_ */
