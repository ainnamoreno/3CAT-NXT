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

double x_i0;
double x_e0;
double x_n0;
double x_M0;
double x_w0;
double x_f0;


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

int sign(double x){

	if(x<0){
		return -1;
	}
	if(x>=0){
		return 1;
	}
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

void rv(double fk, double ik, double uk, double rk, double rDot, double rFDot, double *x){

	x[0] = cos(fk); //cf
	x[1] = sin(fk); //sf
	x[2] = cos(ik); //cI
	x[3] = sin(ik); //sI
	x[4] = -x[1]*x[2]; //M[0]
	x[5] = x[0]*x[2]; //M[1]
	x[6] = x[3]; //M[2]
	x[7] = x[0]; //N[0]
	x[8] = x[1]; //N[1]
	x[9] = 0;    //N[2]
	x[10] = cos(uk); //cUK
	x[11] = sin(uk); //sUK
	x[12] = x[4]*x[11]+x[5]*x[11]+x[6]*x[11]+x[7]*x[10]+x[8]*x[10]+x[9]*x[10]; //U
	x[13] = x[4]*x[10]+x[5]*x[10]+x[6]*x[10]-x[7]*x[11]-x[8]*x[11]-x[9]*x[11]; //V
	x[14] = rk*x[12]; //r
	x[15] = rDot*x[12]+rFDot*x[13]; //v

}




void sgp(double *tVec, int nPts, double y_r[3][nPts], double y_v[3][nPts]){

	double eps;
	double kE = 0.743669161e-1;
	double aE = 1.0;
	double j2 = 1.082616e-3;
	double j3 = -0.253881e-5;
	double twoThirds = 2/3;
	double degtoRad = 3.1415926535897/180;
	double e;

	// Values independent of time since epoch
	double p;
	double fS0;
	double wS0;
	double LS;
	double aYNSL;
	double a;
	double dT;
	double aXNSL;
	double x_n0DDot;
	double x_n0Dot;
	double L;
	double cI0  = cos(x_i0);
	double sI0  = sin(x_i0);
	double e0Sq = pow(x_e0,2);
	double a1   = pow((kE/x_n0),twoThirds);
	double d1   = 0.75*j2*pow((aE/a1),2)*(3*pow(cI0,2) - 1)/pow(1 - e0Sq, 1.5);
	double a0   = a1*(1 - d1/3 - pow(d1,2) - (134/81)*pow(d1,3));
	double p0   = a0*(1 - e0Sq);
	double q0   = a0*(1 - x_e0);
	double L0   = x_M0 + x_w0 + x_f0;
	double z    = 3*j2*pow(aE/p0,2)*x_n0;
	double dFDT = -z*cI0/2;
	double dWDT =  z*(5*pow(cI0,2) - 1)/4;
	double tol  = eps;


	for(int k =0; k<nPts; k++){
		dT = tVec[k];
		a = a0*pow((x_n0/(x_n0 + (2*x_n0Dot + 3*x_n0DDot*dT)*dT)),twoThirds);
		if( a > q0 ){
			e = 1 - q0/a;
		}else{
			e = 1e-6;
		}
		p     = a*(1 - pow(e,2));
		fS0   = x_f0 + dFDT*dT;
		wS0   = x_w0 + dWDT*dT;
		LS    = L0 + (x_n0 + dWDT + dFDT)*dT + pow(x_n0Dot*dT,2) + pow(x_n0DDot*dT,3);
		z     = 0.5*(j3/j2)*aE*sI0/p;
		aYNSL = e*sin(wS0) - z;
		aXNSL = e*cos(wS0);
		L     = FMod2Pi(LS - 0.5*z*aXNSL*(3 + 5*cI0)/(1 + cI0));
		double u      = FMod2Pi(L - fS0);
		//SOLVE KEPLER'S EQUATION
		double ePW    = Kepler( u, aYNSL, aXNSL, tol );
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
		double iK     = x_i0 + 0.75 *z*sI0*cI0*cos2U;
		double *y;
		RV( fK, iK, uK, rK, rDot, rFDot, y );
		y_r[0][k] = y[14];
		y_v[0][k] = y[15];
	}





}
