/*
 * igrf.h
 *
 *  Created on: 14 mar. 2022
 *      Author: jose_
 */

#ifndef INC_IGRF_H_
#define INC_IGRF_H_


//Read spherical harmonic coefficients from model into array
int getshc(const char *file,int iflag,long int strec,int nmax_of_gh,int gh);
//Extrapolate model
int extrapsh(float date);
//Interpolate between models
int interpsh(float date,float dte1,int nmax1,float dte2,int nmax2);
//Calculates field components from models
int shval3(float flat,float flon,float elev,int nmax,float *dest);


#endif /* INC_IGRF_H_ */
