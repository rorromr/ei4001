#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Parabolic interpolation
"""
__author__ = "Rodrigo Muñoz"

import numpy as np

def get_coef(tf, qf, qqf=0, q0=0, qq0=0):
	a2=(3.0*(qf-q0)-tf*(2*qq0+qqf))/(tf**2)
	a3=(2.0*(q0-qf)+tf*(qq0+qqf))/(tf**3)
	return a2, a3

def get_parabolic_trajectory(tf, qf, qqf=0, q0=0, qq0=0, segments=50):
	# Polynomial coeficients
	a0=q0
	a1=qq0
	a2=(3.0*(qf-q0)-tf*(2*qq0+qqf))/(tf**2)
	a3=(2.0*(q0-qf)+tf*(qq0+qqf))/(tf**3)
	# Time vector
	t=np.linspace(0,tf,segments)
	t2=t*t
	t3=t2*t
	return (a0+a1*t+a2*t2+a3*t3, a1+2*a2*t+3*a3*t2, 2*a2+6*a3*t)


print get_parabolic_trajectory(0.5,9.5)
