#pragma once
#include "interpolation.h"

namespace Interpolation
{
	/*
	Returns a parameterized spline function 
	which intersects the given set of points.

	A natural cubic spline is a piece-wise 3rd-order polynomial
		1. intersects all points given to it
		2. second derivatives match where each piece connects
		3. (if open) has a second derivative of 0 at endpoints 

	closed: is the spline closed, i.e. does it make a loop?
	If closed, then the spline can be interpolated farther than t=n-1
	*/
	InterpolationFunc CubicNaturalSplineFunc(bool closed);
};