/*
 * sgp.h
 *
 *  Created on: 18 mar. 2022
 *      Author: jose_
 */

#ifndef INC_SGP_H_
#define INC_SGP_H_

double FMod2Pi(double x);

void sgp(double *tVec, int nPts, double y_r[3][nPts], double y_v[3][nPts]);

int sign(double x);

double kepler(double u, double aYNSL, double aXNSL, double tol );

void rv(double fk, double ik, double uk, double rk, double rDot, double rFDot, double *x);


#endif /* INC_SGP_H_ */
