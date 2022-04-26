/*
 * sgp.c
 *
 *  Created on: 18 mar. 2022
 *      Author: jose_
 */
#include "string.h"
#include "stdlib.h"
#include "sgp.h"
#include "math.h"
#include "satutl.h"
#define NPTS 100

double* linspace(double x1, double x2, int n) {

 double *x = calloc(n, sizeof(double));

 double step = (x2 - x1) / (double)(n - 1);

 for (int i = 0; i < n; i++) {
     x[i] = x1 + ((double)i * step);
 }

return x;
}

double FMod2Pi(double x){
	double y;
	double kpi   = 3.14159265358979323846;
	double twoPi = 2*kpi;
	double k;
	k = floor(x/twoPi);
	y = x - k*twoPi;
	if( y < 0 ){
		y = y + twoPi;
	}
	return y;
}


double kepler(double u, double aYNSL, double aXNSL, double tol ){

	double c, s;
	double ePW = u;
	double delta = 1;
	double ktr = 1;
	while(abs(delta/ePW)>tol){
		c = cos(ePW);
		s = sin(ePW);
		delta = (u - aYNSL*c + aXNSL*s - ePW)/(1 - aYNSL*s - aXNSL*c);
		if(abs(delta)>=0.95){
			delta = 0.95*sign(delta);
		}
		ePW = ePW + delta;
		ktr = ktr + 1;
	}
	return ePW;
}

void rv(float fk, float ik, float uk, float rk, float rDot, float rFDot, float y[3][100]){

	float x[14], pos[3], pos2[3];
	x[0] = cos(fk); //cf
	x[1] = sin(fk); //sf
	x[2] = cos(ik); //cI
	x[3] = sin(ik); //sI
	pos[0] = -x[1]*x[2]; //M[0]
	pos[1] = x[0]*x[2]; //M[1]
	pos[2] = x[3]; //M[2]
	x[4] = x[0]; //N[0]
	x[5] = x[1]; //N[1]
	x[6] = 0;    //N[2]
	x[7] = cos(uk); //cUK
	x[8] = sin(uk); //sUK
	pos2[0] = pos[0]*x[8]+x[4]*x[7]; //U
	pos2[1] = pos[1]*x[8]+x[5]*x[7];
	pos2[2] = pos[2]*x[8]+x[6]*x[7];
	y[0][NPTS] = pos2[0]*rk;
	y[1][NPTS] = pos2[1]*rk;
	y[2][NPTS] = pos2[2]*rk;

}




void sgp(orbit_t orbit, double *tVec, int nPts, float y_r[3][nPts]){


	double eps = pow(2,-52);
	double kE = 0.743669161e-1;
	double aE = 1.0;
	double j2 = 1.082616e-3;
	double j3 = -0.253881e-5;
	double twoThirds = 2/3;
	// Values independent of time since epoch
	double p, fS0, wS0, LS, aYNSL, a, dT, aXNSL, L, e;
	double cI0  = cos(orbit.eqinc);
	double sI0  = sin(orbit.eqinc);
	double e0Sq = pow(orbit.ecc,2);
	double a1   = pow((kE/orbit.rev),twoThirds);
	double d1   = 0.75*j2*pow((aE/a1),2)*(3*pow(cI0,2) - 1)/pow(1 - e0Sq, 1.5);
	double a0   = a1*(1 - d1/3 - pow(d1,2) - (134/81)*pow(d1,3));
	double p0   = a0*(1 - e0Sq);
	double q0   = a0*(1 - orbit.ecc);
	double L0   = orbit.mnan + orbit.argp + orbit.ascn;
	double z    = 3*j2*pow(aE/p0,2)*orbit.rev;
	double dFDT = -z*cI0/2;
	double dWDT =  z*(5*pow(cI0,2) - 1)/4;
	double tol  = eps;


	for(int k =0; k<nPts; k++){
		dT = tVec[k];
		a = a0*pow((orbit.rev/(orbit.rev + (2*orbit.n0Dot + 3*orbit.n0DDot*dT)*dT)),twoThirds);
		if( a > q0 ){
			e = 1 - q0/a;
		}else{
			e = 1e-6;
		}
		p     = a*(1 - pow(e,2));
		fS0   = orbit.ascn + dFDT*dT;
		wS0   = orbit.argp + dWDT*dT;
		LS    = L0 + (orbit.rev + dWDT + dFDT)*dT + pow(orbit.n0Dot*dT,2) + pow(orbit.n0DDot*dT,3);
		z     = 0.5*(j3/j2)*aE*sI0/p;
		aYNSL = e*sin(wS0) - z;
		aXNSL = e*cos(wS0);
		L     = FMod2Pi(LS - 0.5*z*aXNSL*(3 + 5*cI0)/(1 + cI0));
		double u      = FMod2Pi(L - fS0);
		//SOLVE KEPLER'S EQUATION
		double ePW    = kepler( u, aYNSL, aXNSL, tol );
		double c      = cos(ePW);
		double s      = sin(ePW);
		double eCosE  = aXNSL*c + aYNSL*s;
		double eSinE  = aXNSL*s - aYNSL*c;
		double eLSq   = pow(aXNSL,2) + pow(aYNSL,2);
		double pL     = a*(1 - eLSq);
		double r      = a*(1 - eCosE);
		double rDot   = kE*sqrt(a)*eSinE/r;
		double rFDot  = kE*sqrt(pL)/r;
		z      = eSinE/(1 + sqrt(1-eLSq));
		double sinU   = (a/r)*(s - aYNSL - aXNSL*z);
		double cosU   = (a/r)*(c - aXNSL + aYNSL*z);
		u      = atan2(sinU,cosU);
		double cos2U  = 2*pow(cosU,2) - 1;
		double sin2U  = 2*sinU*cosU;
		z      = j2*pow((aE/pL),2);
		double rK     = r    + 0.25 *z*pL*pow(sI0,2)*cos2U;
		double uK     = u    - 0.125*z*(7*pow(cI0,2) - 1)*sin2U;
		double fK     = fS0  + 0.75 *z*cI0*sin2U;
		double iK     = orbit.eqinc + 0.75 *z*sI0*cI0*cos2U;
		rv( fK, iK, uK, rK, rDot, rFDot, y_r);

	}





}
