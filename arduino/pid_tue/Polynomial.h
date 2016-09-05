/** 
 * @file Polynomial.hpp
 *
 * @author Boris Mrkajic
 * @date March, 2011
 * @version 1.0
 *
 * The Polynomial class represents a polynomial of an arbitrary order 
 * and it is a part of Systems & Control Library.
 */

#ifndef POLYNOMIAL_HPP
#define POLYNOMIAL_HPP

#define maxOrder 20

class Polynomial
{
	private:
		/// Order of a polynomial
		short mnTerms;
		/// Coefficients of a polynomial
		double mpCoefficients[maxOrder];
	public:
		/// Default constructor
		Polynomial ();
		
		/// Copy constructor
		/** 
		@param x polynomial to be copied
		*/
		Polynomial (const Polynomial &x);
		
		/// Constructor
		/**
		Constructs a polynomial with order of the given parameter
		@param nTerms order of a polynomial
		*/
        Polynomial (short nTerms);

		/// Destructor
		~Polynomial ();

		/// Overloaded operator =
		Polynomial & operator= (const Polynomial &);
		
		/// Overloaded operator +
		Polynomial operator+ (const Polynomial &) const;
		Polynomial operator+ (double) const;
		friend Polynomial operator+ (double, const Polynomial &);

		/// Overloaded operator -
		Polynomial operator- (const Polynomial &) const;
		Polynomial operator- (double) const;
		friend Polynomial operator- (double, const Polynomial &);

		/// Overloaded operator *
		Polynomial operator* (const Polynomial &) const;
		Polynomial operator* (double) const;
		friend Polynomial operator* (double, const Polynomial &);

		/// Function for setting a coefficient of a polynomial
		void setTerm (short term, double coefficient);
		
		/// Function for getting a coefficient of a polynomial
		double getTerm (short term) const;

		/// Function for evaluation of a polynomial
		double evaluate (double x) const;
};
#endif
