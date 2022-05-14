/*
 * sgp.h
 *
 *  Created on: 18 mar. 2022
 *      Author: jose_
 */

#ifndef INC_SGP_H_
#define INC_SGP_H_

typedef struct __attribute__ ((__packed__)) orbit_t {
	int ep_year;
	long norb;
	float ep_day;
	float bstar;
	float eqinc;
	float ascn;
	float ecc;
	float argp;
	float mnan;
	float rev;
	long satno;
	float n0Dot;
	float n0DDot;

}orbit_t;





float FMod2Pi(float x);

float* linspace(float x1, float x2, int n);

void sgp(orbit_t orbit, float *tVec, int nPts, float *x, float *y, float *z);

int sign(float x);

float kepler(float u, float aYNSL, float aXNSL, float tol );




#endif /* INC_SGP_H_ */
