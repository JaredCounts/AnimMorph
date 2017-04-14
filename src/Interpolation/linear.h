#pragma once

#include "../linearAlgebra.h"
#include <iostream>
namespace Interpolation
{
	VectorXf Linear(const MatrixXf &values, float t)
	{
		int pieceCount = values.rows();
		RowVector2f interpolate = RowVector2f::Zero(pieceCount);
		
		int tIndex = (int)min(max(t, 0.f), pieceCount-2.f);

		interpolate[tIndex] = 1 - t;
		interpolate[tIndex + 1] = t;

		return (interpolate * values).transpose();
	}
};