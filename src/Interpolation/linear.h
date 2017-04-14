#pragma once

#include "../linearAlgebra.h"
#include <iostream>

namespace Interpolation
{
	
	VectorXf Linear(const MatrixXf &values, float t, bool closed)
	{
		const unsigned int pieceCount = values.rows();
		RowVectorXf interpolate = RowVectorXf::Zero(pieceCount);

		int tIndex = 0;
		if (closed)
		{
			tIndex = (int)t % pieceCount;
		}
		else
		{
			tIndex = (int)min(max(t, 0.f), pieceCount - 2.f);
		}

		float tAdjusted = t - tIndex;

		interpolate[tIndex] = 1 - tAdjusted;
		interpolate[(tIndex + 1) % pieceCount] = tAdjusted;

		return (interpolate * values).transpose();
	};

	VectorXf LinearOpen(const MatrixXf &values, float t)
	{
		return Linear(values, t, false);
	};

	VectorXf LinearClosed(const MatrixXf &values, float t)
	{
		return Linear(values, t, true);
	};
};