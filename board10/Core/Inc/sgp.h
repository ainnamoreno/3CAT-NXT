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
	double ep_day;
	double bstar;
	double eqinc;
	double ascn;
	double ecc;
	double argp;
	double mnan;
	double rev;
	long satno;
	double n0Dot;
	double n0DDot;

}orbit_t;


typedef struct __attribute__ ((__packed__)) pos_vel {

	double r;
	double v;

}pos_vel;


double FMod2Pi(double x);

double* linspace(double x1, double x2, int n);

void sgp(orbit_t orbit, double *tVec, int nPts, double y_r[3][nPts], double y_v[3][nPts], pos_vel *posvel);

int sign(double x);

double kepler(double u, double aYNSL, double aXNSL, double tol );

void rV(double fk, double ik, double uk, double rk, double rDot, double rFDot, pos_vel *posvel);



#endif /* INC_SGP_H_ */
