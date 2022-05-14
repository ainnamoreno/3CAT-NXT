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


int sign(float a){
    if(a < 0.0)
        return -1;
    if(a > 0.0)
        return 1;
    return 0;
}

float* linspace(float x1, float x2, int n) {

 float *x = calloc(n, sizeof(float));

 float step = (x2 - x1) / (float)(n - 1);

 for (int i = 0; i < n; i++) {
     x[i] = x1 + ((float)i * step);
 }

return x;
}

float FMod2Pi(float x){
	float y;
	float kpi   = 3.14159265358979323846;
	float twoPi = 2*kpi;
	float k;
	k = floor(x/twoPi);
	y = x - k*twoPi;
	if( y < 0 ){
		y = y + twoPi;
	}
	return y;
}


float kepler(float u, float aYNSL, float aXNSL, float tol ){

	float c, s;
	float ePW = u;
	float delta = 1;
	float ktr = 1;
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





void sgp(orbit_t orbit, float *tVec, int nPts, float *x, float *y, float *z){


    float p, fS0, wS0, LS, aYNSL, a, dT, aXNSL, L, e, cI0, sI0, e0Sq, a1, d1, a0, p0, q0, L0, aux, dFDT, dWDT, tol, cf, sf, ci, si, cuk, suk;
    float pos[3], pos2[3], pos3[3];
    float orbit_eqinc = 0.901282789742116;
    float orbit_ecc = 1.011000000000000e-04;
    float orbit_rev = 0.068238005150227;
    float orbit_mnan =0.646166267648853;
    float orbit_argp = 0.440203453279506;
    float orbit_ascn = 4.118814719086182;
    float orbit_n0Dot = 2.85176497485150e-09;
    float orbit_n0DDot = 0;
    float eps = powf(2,-52);
	float kE = 0.0743669161;
	float aE = 1.0;
	float j2 = 1.082616e-3;
	float j3 = -0.253881e-5;
	float twoThirds = 2.0/3.0;
	tVec = linspace(0,60,100);
	// Values independent of time since epoch
	cI0  = cos(orbit_eqinc);
	sI0  = sin(orbit_eqinc);
	e0Sq = powf(orbit_ecc,2);
	a1   = pow((kE/orbit_rev),twoThirds);
	d1   = 0.75*j2*powf((aE/a1),2)*(3*powf(cI0,2) - 1)/powf(1 - e0Sq, 1.5);
	a0   = a1*(1 - d1/3 - powf(d1,2) - (134/81)*powf(d1,3));
	p0   = a0*(1 - e0Sq);
	q0   = a0*(1 - orbit_ecc);
	L0   = orbit_mnan + orbit_argp + orbit_ascn;
	aux    = 3*j2*powf(aE/p0,2)*orbit_rev;
	dFDT = -aux*cI0/2;
	dWDT =  aux*(5*powf(cI0,2) - 1)/4;
	tol  = eps;
    float rK, uK, fK, iK, rDot, rFDot, u, ePW, c, s, eCosE, eSinE, eLSq, pL, r, sinU, cosU, sin2U, cos2U;

	for(int k = 0; k<nPts; k++){
		dT = tVec[k];
		a = a0*powf((orbit_rev/(orbit_rev + (2*orbit_n0Dot + 3*orbit_n0DDot*dT)*dT)),twoThirds);
		if( a > q0 ){
			e = 1 - q0/a;
		}else{
			e = 1e-6;
		}
		p     = a*(1 - powf(e,2));
		fS0   = orbit_ascn + dFDT*dT;
		wS0   = orbit_argp + dWDT*dT;
		LS    = L0 + (orbit_rev + dWDT + dFDT)*dT + powf(orbit_n0Dot*dT,2) + powf(orbit_n0DDot*dT,3);
		aux     = 0.5*(j3/j2)*aE*sI0/p;
		aYNSL = e*sin(wS0) - aux;
		aXNSL = e*cos(wS0);
		L     = FMod2Pi(LS - 0.5*aux*aXNSL*(3 + 5*cI0)/(1 + cI0));
		float u      = FMod2Pi(L - fS0);
		//SOLVE KEPLER'S EQUATION
		ePW    = kepler( u, aYNSL, aXNSL, tol );
		c      = cos(ePW);
		s      = sin(ePW);
		eCosE  = aXNSL*c + aYNSL*s;
		eSinE  = aXNSL*s - aYNSL*c;
		eLSq   = powf(aXNSL,2) + powf(aYNSL,2);
		pL     = a*(1 - eLSq);
		r      = a*(1 - eCosE);
		rDot   = kE*sqrt(a)*eSinE/r;
		rFDot  = kE*sqrt(pL)/r;
	    aux     = eSinE/(1 + sqrt(1-eLSq));
		sinU   = (a/r)*(s - aYNSL - aXNSL*aux);
		cosU   = (a/r)*(c - aXNSL + aYNSL*aux);
		u      = atan2(sinU,cosU);
		cos2U  = 2*powf(cosU,2) - 1;
		sin2U  = 2*sinU*cosU;
		aux      = j2*powf((aE/pL),2);
		rK     = r    + 0.25 *aux*pL*powf(sI0,2)*cos2U;
		uK     = u    - 0.125*aux*(7*powf(cI0,2) - 1)*sin2U;
		fK     = fS0  + 0.75 *aux*cI0*sin2U;
		iK     = orbit_eqinc + 0.75 *aux*sI0*cI0*cos2U;
	    cf = cos(fK); //cf
        sf = sin(fK); //sf
	    ci = cos(iK); //cI
	    si = sin(iK); //sI
	    pos[0] = -sf*ci; //M[0]
	    pos[1] = cf*ci; //M[1]
	    pos[2] = si; //M[2]
	    pos3[0] = cf; //N[0]
	    pos3[1] = sf; //N[1]
	    pos3[2] = 0;    //N[2]
	    cuk = cos(uK); //cUK
	    suk = sin(uK); //sUK
	    pos2[0] = pos[0]*suk+pos3[0]*cuk; //U
	    pos2[1] = pos[1]*suk+pos3[1]*cuk;
	    pos2[2] = pos[2]*suk+pos3[2]*cuk;
	    x[k] = pos2[0]*rK*6378.135;
	    y[k] = pos2[1]*rK*6378.135;
	    z[k] = pos2[2]*rK*6378.135;
	}

}
