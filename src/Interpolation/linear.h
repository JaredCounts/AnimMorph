#pragma once

#include "interpolation.h"

namespace Interpolation
{
	/*
	Returns a parameterized linear function which 
	intersects a given set of points.

	closed: should the lines make a loop?
	If closed, then the spline can be interpolated farther than t=n-1
	*/
	InterpolationFunc LinearFunc(bool closed);
};