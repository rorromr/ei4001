/** 
 * @file Polynomial.cpp
 *
 * @author Boris Mrkajic
 * @date March, 2011
 * @version 1.0
 *
 */
 
#include "Polynomial.h"

//constructor
Polynomial::Polynomial() :mnTerms(3)
{
	for ( int i = 0; i < mnTerms; i++ )
		mpCoefficients[i]=0;
}

// copy constructor
Polynomial::Polynomial(const Polynomial &x)
{
    mnTerms=x.mnTerms;
	
	for (int i=0;i<mnTerms;i++)
		mpCoefficients[i]=x.mpCoefficients[i];
}

//constructor
Polynomial::Polynomial(short nTerms)
{
	if (nTerms < maxOrder && nTerms >= 0) {
		mnTerms=nTerms+1;
	} else {
		mnTerms=maxOrder;
	}
	
	for(int i=0;i<mnTerms;i++)
		mpCoefficients[i]=0;
}

//destructor
Polynomial::~Polynomial()
{
}

Polynomial &Polynomial::operator= (const Polynomial &rhs)
{
	if (this==&rhs)
		return *this;

	if (mnTerms!=rhs.mnTerms)
	{
		for(int i=0;i<mnTerms;i++)
			mpCoefficients[i]=0;
		mnTerms=rhs.mnTerms;
	}
	
	for (int i=0;i<mnTerms;i++)
		mpCoefficients[i] = rhs.mpCoefficients[i];

	return *this;
}

Polynomial Polynomial::operator+ (const Polynomial &rhs) const
{
	short biggest=mnTerms;
	
	if (rhs.mnTerms>biggest)
		biggest=rhs.mnTerms;
		
	Polynomial answer(biggest-1);
	for (int i=(biggest-1);i>=0;i--)
	{
		if ((i<mnTerms)&&(i<rhs.mnTerms))
			answer.mpCoefficients[i]=mpCoefficients[i]+rhs.mpCoefficients[i];
		else if (i<mnTerms)
			answer.mpCoefficients[i]=mpCoefficients[i];
		else
			answer.mpCoefficients[i]=rhs.mpCoefficients[i];
	}
	return answer;
}

Polynomial Polynomial::operator+ (double rhs) const
{
	Polynomial answer (*this);
	answer.mpCoefficients[0]+=rhs;
	return answer;
}

Polynomial operator+ (double lhs,const Polynomial &rhs)
{
	Polynomial answer(rhs);
	answer.mpCoefficients[0]+=lhs;
	return answer;
}

Polynomial Polynomial::operator- (const Polynomial &rhs) const
{
	short biggest=mnTerms;

	if (rhs.mnTerms>biggest)
		biggest=rhs.mnTerms;
		
	Polynomial answer(biggest-1);
	for (int i=biggest-1;i>=0;i--)
	{
		if ((i<mnTerms)&&(i<rhs.mnTerms))
			answer.mpCoefficients[i]=mpCoefficients[i]-rhs.mpCoefficients[i];
		else if (i<mnTerms)
			answer.mpCoefficients[i]=mpCoefficients[i];
		else
			answer.mpCoefficients[i]=-rhs.mpCoefficients[i];
	}
	
	return answer;
}

Polynomial Polynomial::operator- (double rhs) const
{
	Polynomial answer (*this);
	answer.mpCoefficients[0]-=rhs;
	return answer;
}

Polynomial operator- (double lhs, const Polynomial &rhs)
{
	Polynomial answer(rhs);
	int num = answer.mnTerms;
	
	for (int i=num-1;i>=0;i--)
		answer.mpCoefficients[i]=-answer.mpCoefficients[i];
		
	answer.mpCoefficients[0]+=lhs;
	return answer;
}

Polynomial Polynomial::operator* (const Polynomial &rhs) const
{
	Polynomial answer(mnTerms+rhs.mnTerms-2);
	
	for (int i=0;i<mnTerms;i++)
		for (int j=0;j<rhs.mnTerms;j++)
			answer.mpCoefficients[i+j]+=mpCoefficients[i]*rhs.mpCoefficients[j];
	
	return answer;
}

Polynomial Polynomial::operator* (double rhs) const
{
	Polynomial answer (*this);
	
	for (int i=0;i<mnTerms;i++)
		answer.mpCoefficients[i]*=rhs;
		
	return answer;
}

Polynomial operator* (double lhs,const Polynomial &rhs)
{
	Polynomial answer(rhs);
	int num = answer.mnTerms;
	
	for (int i=num-1;i>=0;i--)
		answer.mpCoefficients[i]*=lhs;
	
	return answer;
}

void Polynomial::setTerm (short term,double coefficient)
{
	if (term < mnTerms && term >= 0)
		mpCoefficients[term]=coefficient;
}

double Polynomial::getTerm (short term) const
{
	if (term < mnTerms && term >= 0)
		return mpCoefficients[term];
}

double Polynomial::evaluate (double x) const
{
	double answer = 0.0;
	double xx = 1;
	
	for (int i=0;i<mnTerms;i++) {
		
		answer += mpCoefficients[i]*xx;
		xx = xx*x;
	}
		
	return answer;
}
