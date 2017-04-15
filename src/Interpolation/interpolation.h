#include <functional>

#include "../linearAlgebra.h"

namespace Interpolation
{
	/*
	Some function which interpolates along a given set of points.

	Values is an m x n matrix, 
		where rows m = number of axes to interpolate along
		  and cols n = number of values along each axis

	The scalar t is how far to interpolate along the curve,
	with 0 <= t < n

	At t = 0, we get values.col(0),
	   t = 1, values.col(1),
	   ...
	   t = n-1, values.col(n-1)
	   */
	typedef 
		std::function<VectorXf(const MatrixXf &values, float t)> 
		InterpolationFunc;
}