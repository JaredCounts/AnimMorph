#pragma once

#include "interpolation.h"

namespace Interpolation
{
	/*
	Returns a parameterized bezier function which
	intersects the given end points
	*/
	InterpolationFunc BezierFunc(const VectorXf &controlPointA, const VectorXf &controlPointB);
};