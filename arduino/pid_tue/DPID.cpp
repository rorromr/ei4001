/** 
 * @file DPID.cpp
 * 
 * @author Boris Mrkajic
 * @date May, 2011
 * @version 1.0
 *
 * @brief The DPID class represents a digital PID filter and it is 
 * a part of Systems & Control Library (section Digitial Filters).
 */

/************************************************************************
 *	Copyright (C) 2011 Eindhoven University of Technology (TU/e).		*
 *	All rights reserved.												*
*************************************************************************
 *	Redistribution and use in source and binary forms, with or without	*
 *	modification, are permitted provided that the following conditions	*
 *	are met:															*
 *																		*
 *		1.	Redistributions of source code must retain the above		*
 * 			copyright notice, this list of conditions and the following *
 * 			disclaimer.													*
 *																		*
 *		2. 	Redistributions in binary form must reproduce the above		*
 *			copyright notice, this list of conditions and the following *
 *			disclaimer in the documentation and/or other materials 		*
 *			provided with the distribution.								*
 *																		*
 *	THIS SOFTWARE IS PROVIDED BY TU/e "AS IS" AND ANY EXPRESS OR 		*
 *	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 		*
 *	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE	*
 *	ARE DISCLAIMED. IN NO EVENT SHALL TU/e OR CONTRIBUTORS BE LIABLE 	*
 *	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 		*
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 	*
 *	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 	*
 *	OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 		*
 *	LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 			*
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 	*
 *	USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH	* 
 *	DAMAGE.																*
 *																		*
 *	The views and conclusions contained in the software and 			*
 *	documentation are those of the authors and should not be 			*
 *	interpreted as representing official policies, either expressed or 	*
 *	implied, of TU/e.													*
 ************************************************************************/

#include "DPID.h"

using namespace DFILTERS;

DPID::DPID() :eps(1e-8)
{
	initialize();
}

DPID::DPID(const DPID &x)
{
	denominator[0]=x.denominator[0];
	numerator[0]=x.numerator[0];
	for ( int i = 0; i < filter_order; i++ ) {
		previous_inputs[i]  = x.previous_inputs[i];
		previous_outputs[i] = x.previous_outputs[i];		
		denominator[i+1] = x.denominator[i+1];
		numerator[i+1] = x.numerator[i+1];	
	}
	eps = x.eps;
}

DPID::DPID (float kp, float kv, float ki, float _Ts, int method, float _limit, float _kaw) :eps(1e-8)
{
	initialize();
	configure(kp, kv, ki, _Ts, method, _limit, _kaw);
}

DPID::~DPID()
{
	finalize();
}


bool DPID::initialize()
{
	denominator[0] = 1.0;
	numerator[0] = 1.0;	
	for ( int i = 0; i < filter_order; i++ ) {
		denominator[i+1] = 0.0;
		numerator[i+1] = 0.0;	
		previous_inputs[i]  = 0.0;
		previous_outputs[i] = 0.0;		
	}
	
	output = 0.0;
	output_antiwindup = 0.0;
	anti_windup = 0.0;
	windup = 0.0;
	previous_antiwindup = 0.0;
	previous_windup = 0.0;

	deadZoneEn_ = false;
    isDeadZone_ = false;
	
	return true;
}

bool DPID::configure(float kp, float kv, float ki, float _Ts, int method, float _limit, float _kaw)
{
	// Ensure valid input parameters and cast paramter method from int to DiscretizationMethod type (required for enumeration check)
	if ( (fabs(kp) < eps && fabs(kv) < eps) || fabs(_Ts) < eps || method < 1 || method > 5)
		return false;

	Ts = _Ts;
	enum_method = DiscretizationMethod(method);
	limit = fabs(_limit);
	kaw = fabs(_kaw);

	float N = 100.0;
	float alpha, wp;
	float zp1=1.0;
	float zp2=exp(-N*Ts);
	float zz1=exp(Ts*(-kp*N-ki+sqrt(pow(kp*N+ki,2)-4*ki*(kp+kv*N)))/(2*(kp+kv*N)));
	float zz2=exp(Ts*(-kp*N-ki-sqrt(pow(kp*N+ki,2)-4*ki*(kp+kv*N)))/(2*(kp+kv*N)));

	float bD[filter_order+1], aD[filter_order+1];
	
	Polynomial numCont(filter_order);
	Polynomial denCont(filter_order);
	
	numCont.setTerm(2,kp+kv*N);	//b=(kp+kv*N)*s^2+(kp*N+ki)*s+ki*N
	numCont.setTerm(1,kp*N+ki);
	numCont.setTerm(0,ki*N);
	denCont.setTerm(2,1.0);		//a=s^2+N*s
	denCont.setTerm(1,N);
	denCont.setTerm(0,0.0);

	Polynomial p(filter_order);
	Polynomial q(filter_order);

	switch (enum_method){
		case EulerBackward: 
			// Euler - backward differentiation method
			// s ~ p(z)/q(z)
			p.setTerm(1,1);			//p(z)=z-1
			p.setTerm(0,-1);
			q.setTerm(1,Ts);		//q(z)=Ts*z
			q.setTerm(0,0);
			
			cont2discrete(aD, bD, p, q, denCont, numCont);
			break;
		case EulerForward:
			// Euler - forward differentiation method
			// s ~ p(z)/q(z)
			p.setTerm(1,1);			//p(z)=z-1
			p.setTerm(0,-1);
			q.setTerm(1,0);			//q(z)=Ts
			q.setTerm(0,Ts);
			
			cont2discrete(aD, bD, p, q, denCont, numCont);
			break;
		case Tustin:
			// Tustin method
			alpha = 2/Ts;
			// s ~ p(z)/q(z)
			p.setTerm(1,alpha);		//p(z)=alpha*z-alpha
			p.setTerm(0,-alpha);
			q.setTerm(1,1);			//q(z)=z+1
			q.setTerm(0,1);
			
			cont2discrete(aD, bD, p, q, denCont, numCont);
			break;
		case TustinPrewarp:
			// Tustin with prewarping method
			wp = 2*M_PI*exp(Ts*(-kp*N-ki-sqrt(pow(kp*N+ki,2)-4*ki*(kp+kv*N)))/(2*(kp+kv*N)));
			alpha = wp/(tan(wp*Ts/2)); 
	
			p.setTerm(1,alpha);		//p=alpha*z-alpha
			p.setTerm(0,-alpha);
			q.setTerm(1,1);			//q=z+1
			q.setTerm(0,1);
			
			cont2discrete(aD, bD, p, q, denCont, numCont);
			break;
		case ZOH:
			// Zero-order hold (ZOH) method
			aD[0] = 1.0;
			aD[1] = -1-exp(-N*Ts);
			aD[2] = exp(-N*Ts);
			bD[0] = kp+kv*N;
			bD[1] = -kp-kp*exp(-N*Ts)-2*kv*N+ki*Ts;
			bD[2] = kp*exp(-N*Ts)+kv*N-ki*Ts*exp(-N*Ts);
			break;			
		default:
			// Tustin with prewarping method
			wp = 2*M_PI*exp(Ts*(-kp*N-ki-sqrt(pow(kp*N+ki,2)-4*ki*(kp+kv*N)))/(2*(kp+kv*N)));
			alpha = wp/(tan(wp*Ts/2)); 
			
			// s ~ p(z)/q(z)
			p.setTerm(1,alpha);		//p(z)=alpha*z-alpha
			p.setTerm(0,-alpha);
			q.setTerm(1,1);			//q(z)=z+1
			q.setTerm(0,1);
			
			cont2discrete(aD, bD, p, q, denCont, numCont);
			break;
	}
	
	bool result = coefficientNormalization(aD, bD);
	return result;
}

bool DPID::update(float input)
{
	output  = numerator[0] * input;
	for ( int i = 1; i <= filter_order; i++ ) {
		output += numerator[i] * previous_inputs[i-1];
		output -= denominator[i] * previous_outputs[i-1];
	}
	savePreviousIO(input, output);
	antiWindup();

	// Dead zone
    if (deadZoneEn_ && (input < deadZone_ && input > -deadZone_))
    {
    	output = 0;
        isDeadZone_ = true;
    }
	
	return true;
}

bool DPID::finalize()
{
	denominator[0]=1.0;
	numerator[0]=1.0;
	for ( int i = 0; i < filter_order; i++ ) {
		previous_inputs[i]  = 0.0;
		previous_outputs[i] = 0.0;		
		denominator[i+1] = 0.0;
		numerator[i+1] = 0.0;
	}
	output = 0.0;
	
	return true;
}

bool DPID::restart()
{
	for ( int i = 0; i < filter_order; i++ ) {
		previous_inputs[i]  = 0.0;
		previous_outputs[i] = 0.0;		
	}
	output = 0.0;
	
	return true;
}

float* DPID::getNumerator () 
{
	return numerator;
}

float* DPID::getDenominator ()
{
	return denominator;
}

float* DPID::getPreviousInputs ()
{
	return previous_inputs;
}

float* DPID::getPreviousOutputs ()
{
	return previous_outputs;
}

float DPID::getOutput ()
{
	return output;
}

float DPID::getOutputAntiwindup ()
{
	return output_antiwindup;
}

bool DPID::setEpsilon (float epsilon)
{
	// Ensure nonzero value for eps
	if (epsilon == 0)
		return false;
	
	eps = fabs(epsilon);
	return true;
}


void DPID::setDeadZone(float error)
{
  deadZone_ = error;
}

void DPID::enableDeadZone(bool enable)
{
  deadZoneEn_ = enable;
}

bool DPID::isDeadZone()
{
  return isDeadZone_;
}

float DPID::getLimit(){
	return limit;
}

void DPID::setLimit(float _limit){
	limit = fabs(_limit);
}


void DPID::savePreviousIO(float in, float out)
{
	for ( int i = filter_order-1; i > 0; i-- ) {
		previous_outputs[i] = previous_outputs[i-1];
		previous_inputs[i]  = previous_inputs[i-1];
	}
	previous_outputs[0] = out;
	previous_inputs[0]  = in;
}

void DPID::cont2discrete(float* denDiscrete, float* numDiscrete, Polynomial &p, Polynomial &q, Polynomial &denCont, Polynomial &numCont) 
{
	Polynomial polynom_den(filter_order);
	Polynomial polynom_num(filter_order);
	
	// Transformation from s to z domain of numerator
	for (int i = 0; i <= filter_order ; i++) {
		Polynomial pp(0);
		Polynomial qq(0);
		pp.setTerm(0,1);
		qq.setTerm(0,1);
		for (int j = 0; j < filter_order - i ; j++) {
			pp = pp*p;
		} 
		for (int k = 0; k < i ; k++) {
			qq = qq*q;
		} 
		polynom_den = polynom_den + (numCont.getTerm(filter_order-i)*pp*qq);
	}
	
	// Transformation from s to z domain of denominator
	for (int i = 0; i <= filter_order ; i++) {
		Polynomial pp(0);
		Polynomial qq(0);
		pp.setTerm(0,1);
		qq.setTerm(0,1);
		for (int j = 0; j < filter_order - i ; j++) {
			pp = pp*p;
		} 
		for (int k = 0; k < i ; k++) {
			qq = qq*q;
		} 
		polynom_num = polynom_num + (denCont.getTerm(filter_order-i)*pp*qq);
	}
	
	// Formation of filter numerator and denonimator (not normilized yet)
	for (int i = 0; i <= filter_order ; i++) {
		numDiscrete[i] = polynom_den.getTerm(filter_order-i);
		denDiscrete[i] = polynom_num.getTerm(filter_order-i);
	}
}

bool DPID::coefficientNormalization(float* den, float* num) 
{
	if (fabs(den[0]) < eps) {
		return false;
	}
		
	for (int i = 0; i <= filter_order ; i++) {
		denominator[i] = den[i]/den[0];
		numerator[i] = num[i]/den[0];
	}
	return true;
}

bool DPID::antiWindup()
{
	float bD[2], aD[2];
	float alpha, wp;
	
	switch (enum_method){
		case EulerBackward: 
			// Euler - backward differentiation method
			aD[0] = 1.0;
			aD[1] = -1.0;
			bD[0] = kaw*Ts;
			bD[1] = 0.0;
			break;
		case EulerForward:
			// Euler - forward differentiation method
			aD[0] = 1.0;
			aD[1] = -1.0;
			bD[0] = 0.0;
			bD[1] = kaw*Ts;
			break;
		case Tustin:
			// Tustin method
			alpha = 2/Ts;
			
			aD[0] = 1.0;
			aD[1] = -1.0;
			bD[0] = kaw/alpha;
			bD[1] = kaw/alpha;
			break;
		case TustinPrewarp:
			// Tustin with prewarping method
			wp = 1.0;
			alpha = wp/(tan(wp*Ts/2)); 

			aD[0] = 1.0;
			aD[1] = -1.0;
			bD[0] = kaw/alpha;
			bD[1] = kaw/alpha;
			break;
		case ZOH:
			// Zero-order hold (ZOH) method
			aD[0] = 1.0;
			aD[1] = -1.0;
			bD[0] = 0.0;
			bD[1] = kaw*Ts;
			break;			
		default:
			// Euler - backward differentiation method
			aD[0] = 1.0;
			aD[1] = -1.0;
			bD[0] = kaw*Ts;
			bD[1] = 0.0;
			break;
	}
	
	anti_windup  = bD[0] * windup;
	anti_windup += bD[1] * previous_windup;
	anti_windup -= aD[1] * previous_antiwindup;
	
	previous_windup = windup;
	previous_antiwindup = anti_windup;

	// Value of the output before saturation block
	float output_tmp = output + anti_windup;
	
	// Final value of the output
	output_antiwindup = output_tmp;
	if (limit > 0.0) {
		if (output_tmp > limit) {
			output_antiwindup = limit;
		}
		if (output_tmp < -limit) {
			output_antiwindup = -limit;
		}
	}

	// Windup value
	windup = output_antiwindup - output_tmp;
	return true;
}
