/** 
 * @file DPID.hpp
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

#ifndef DPID_HPP
#define DPID_HPP

#include <math.h>
#include "scl/filters/DiscretizationMethods.hpp"
#include "scl/polynomial/Polynomial.hpp"

namespace DFILTERS {

/** Digital Leadlag Filter
 *
 *  The DPID class represents a digital PID filter and it is a part of 
 *  Systems & Control Library (section Digitial Filters).
 * 
 *  The name is rather straighforward, with the leading D as it 
 *  represents digital implementation of a filter.
 * 
 *  A general digital filter is characterized by its (discrete-time) 
 *  transfer function. A PID filter is on the other hand characterized by 
 *  the three general parameters, proportional coefficient kp,
 *  derivative coefficient kv and integral coefficient ki. Usually, PID 
 *  filter has two zeros and one pole (it is non-proper). However, this 
 *  makes the implementation difficult and yields not the best results. 
 *  Thus, another parameter is introduced, and it is filter 
 *  coefficient N, which determines the additional pole location of the 
 *  filter. In order to obtain a discrete-time transfer function, an 
 *  another paramter is needed, which is sampling time.
 *  
 *  Additional functionallity of PID is enabled as well. It is an
 *  anti-windup term and it can be activated by setting limit (control
 *  input limit) and kaw (anti-windup coefficient). Back Calculation 
 *  Anti-wind method has been implemented (for more information, see
 *  Åström:2002). Integration method is the same as the discretization
 *  method chosen for the entire filter.
 * 
 *  For the implemenation, the transposed-direct-form II (TDF-II) 
 *  structure is chosen. An advantage of this structure is that the zeros 
 *  effectively precede the poles in series order. Additonally, the 
 *  transposition does not modify the transfer function of Single-Input, 
 *  Single-Output (SISO) filter (see Oppenheim:1975).
 *  
 *  Various discretization methods are available, namely Euler backward 
 *  differentiation method, Euler forward differentiation method, 
 *  standard Tustin method, Tustin method with prewarping (default),
 *  zero-order hold method (see Haugen:2009, Ogata:1987 and Yang:2009). 
 *  Method can be chosen at the moment of filter creation or through 
 *  configuration function.
 *  
 *  DPID class uses the Polynomial class. The motivation yields from
 *  the way discretization is done, since most of the methods function 
 *  such that they only introduce replacement of s (from s-domain) with 
 *  a function of z (from z-domain), i.e. s~f(z). This makes it possible 
 * 	to generalize the routine and to make the derivation of discrete-time 
 * 	transfer function from the continuous-time transfer function automatic.
 * 	
 *	@verbatim
 *	References:
 *	@book{Oppenheim:1975,
 *	author = {Oppenheim, Alan V. and Schafer},
 *	title = {Discrete-time signal processing},
 *	year = {1975},
 *	publisher = {Prentice-Hall, Inc.},
 *	address = {Englewood Cliffs, NJ, USA},
 *	}
 *	
 *	@book{Haugen:2009,
 *	author = "Finn Haugen",
 *	title = "Lecture Notes in Models, Estimation and Control",
 *	year = 2009,
 *	publisher = "TechTeach",
 *	ISBN = "978-82-91748-14-6",
 *	address = "Skien, Norway",
 *	}
 *	
 *	@book{Ogata:1987,
 *	author = {K. Ogata},
 *	title = {Discrete-Time Control Systems},
 *	year = {1987},
 *	publisher = {Prentice-Hall, Inc.},
 *	address = {Englewood Cliffs, NJ, USA},
 *	}
 * 		
 *	@book{Yang:2009,
 *	author = "Won Young Yang",
 *	title = "Signals and Systems with MATLAB",
 *	year = 2009,
 *	publisher = "Springer Berlin Heidelberg",
 *	ISBN = "978-3-540-92954-3",
 *	address = "Berlin, Germany",
 *	}
 *	@endverbatim
 */

	
	class DPID
	{
		public:
		
			/// Order of a PID filter by definition equals two (it consists of two zeros and two poles - in our case)
			static const int filter_order = 2;

			/// Default constructor
			/** 
			Constructor that creates a filter with transfer function
			that equals 1 (no filtering)
			*/
			DPID ();
			
			/// Copy constructor
			/** 
			Constructor that copies a filter together with all its 
			features
			*/
			DPID (const DPID &);
			
			/// Constructor
			/** 
			Constructor that creates a leadlag filter with the given
			parameters
			@param kp proportional coefficient
			@param kv derivative coefficient
			@param ki integral coefficient
			@param Ts sampling time used for filter discretization
			@param method discretization method [default = 4]
			(1- Euler backward, 2- Euler forward, 3- Tustin, 
			4- Prewarp Tustin, 5- Zero-order hold, 6- Zero-pole matching)
			*/
			DPID (double kp, double kv, double ki, double _Ts, int method=4, double _limit=0.0, double _kaw=0.0);
			
			/// Destructor
			/** 
			Destructor that finalizes, i.e. resets parameters of a filter
			*/
			~DPID ();
			
			/// Filter initialization
			/**
			Sets the parameters such that the transfer function equals 1 
			(no filtering) and output 0
			*/
			bool initialize();
			
			/// Filter configuration
			/**
			Configures filter with the given parameters. Discratization 
			method may be left out (default method, Tustin with 
			prewarping, will be used)
			@param kp proportional coefficient
			@param kv derivative coefficient
			@param ki integral coefficient
			@param Ts sampling time used for filter discretization
			@param method discretization method [default = 4]
			(1- Euler backward, 2- Euler forward, 3- Tustin, 
			4- Prewarp Tustin, 5- Zero-order hold, 6- Zero-pole matching)
			*/
			bool configure(double kp, double kv, double ki, double _Ts, int method=4, double _limit=0.0, double _kaw=0.0);
			
			/// Filter update
			/** 
			Function used for update of the current filter output, 
			depending on the given current input
			@param input current filter input
			*/
			bool update(double input);
			
			/// Filter finalization
			/**
			Sets the parameters such that the transfer function equals 1 
			(no filtering) and output 0
			*/
			bool finalize();
			
			/// Get numerator of the filter
			/** @return numerator of the filter as an array pointer
			*/
			double* getNumerator ();
			
			/// Get denominator of the filter
			/** @return denominator of the filter as an array pointer
			*/
			double* getDenominator ();
			
			/// Get previous inputs of the filter
			/** @return previous inputs of the filter as an array pointer
			*/
			double* getPreviousInputs ();
			
			/// Get previous outputs of the filter
			/** @return previous outputs of the filter as an array pointer
			*/
			double* getPreviousOutputs ();	
			
			/// Get current output of the filter
			/** @return current output of the filter
			*/
			double getOutput ();
			
			/// Get current output of the filter with anti-windup
			/** @return current output of the filter with anti-windup
			*/
			double getOutputAntiwindup ();
			
			/// Set the epsilon, value used to denote a number very close to zero (or some other specific value) within the bounds of double accuracy
			bool setEpsilon(double epsilon);
			
		private:
			/// Denominator (coefficients) of a filter
			double denominator[filter_order+1];
			/// Numerator (coefficients) of a filter			
			double numerator[filter_order+1];

			/// Previous inputs of a filter
			double previous_inputs[filter_order];
			/// Previous outputs of a filter			
			double previous_outputs[filter_order];
			
			/// Output of a filter
			double output;

			/// Choice of the discretization method
			DiscretizationMethod enum_method;
			
			/// Sampling time
			double Ts;
			
			/// Value used to denote a number very close to zero (or some other specific value) within the bounds of double accuracy [default = 1e-8]
			double eps;
			
			/// Anti-windup value, basically an output of anti-windup integrator (which compasates for windup of the integrator integrated in PID filter)
			double anti_windup;

			/// Windup value, the difference between saturated and non-saturated value of the output. Also, it is an input to the integrator
			double windup;
			
			/// Previuos anti-windup value, needed for the implementation of the integrator
			double previous_antiwindup;

			/// Previuos windup value, needed for the implementation of the integrator
			double previous_windup;
			
			/// Output of the PID with anti-windup (saturated)
			double output_antiwindup;
			
			/// Limit of the saturation term, i.e. control input limits
			double limit;
			
			/// Anti-windup coefficient
			double kaw;
										
			/// Update of a filter IO history, required for implementation
			/** 
			@param in current input
			@param out current output
			*/
			void savePreviousIO(double in, double out);
			
			/// Discretization of a filter transfer function
			/** 
			@param denDiscrete discretized (z-domain) denominator (not normalized yet)
			@param numDiscrete discretized (z-domain) numerator (not normalized yet)
			@param p (s->z domain) transformation numerator
			@param q (s->z domain) transformation denominator
			@param denCont s-domain denominator
			@param numCont s-domain numerator
			*/
			void cont2discrete(double* denDiscrete, double* numDiscrete, Polynomial &p, Polynomial &q, Polynomial &denCont, Polynomial &numCont);
			
			/// Normalization of denominator and numerator (coefficients) of a filter discrete-time transfer function, required for implementation
			/** 
			Normalization in this case consides making the first coefficient 
			of the transfer function denominator one, i.e. dividing 
			all the coefficients of the denominator and numerator by it.
			This is required for feasability of the filter implementation.
			@param den discretized (z-domain) denominator (not normalized yet)
			@param num discretized (z-domain) numerator (not normalized yet)
			*/
			bool coefficientNormalization(double* den, double* num);
			
			/// Anti-windup part of PID
			/** 
			Function that adds anti-windup functionallity to the PID
			filter. It calculates output_antiwindup and it will differ
			from output value only if the values kaw and limit are set, 
			i.e. anti-windup term used.
			*/
			bool antiWindup();
				
	};

}
#endif
