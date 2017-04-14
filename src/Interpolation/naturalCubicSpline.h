#pragma once

#include "../linearAlgebra.h"
#include <iostream>

namespace Interpolation
{
	float CubicNaturalScalar(const VectorXf &values, float t, bool closed)
	{
		int n = values.size();

		if (n == 0)
		{
			return 0;
		}
		if (n == 1)
		{
			return values[0];
		}

		MatrixXf integralSystem
			= MatrixXf::Zero(n,n);

		VectorXf secondDifferences(n);

		for (int i = 0; i < n; i++)
		{
			if (i == 0)
			{
				if (closed)
				{
					integralSystem(i, i) = 4;
					integralSystem(i, i + 1) = 1;
					integralSystem(i, n) = 1;
					secondDifferences(i) = 3 * (values(i + 1) - values(i));
				}
				else
				{
					integralSystem(i, i) = 2;
					integralSystem(i, i + 1) = 1;
					secondDifferences(i) = 3 * (values(i + 1) - values(i));
				}
			}
			else if (i == n-1)
			{
				if (closed)
				{
					integralSystem(i, i - 1) = 1;
					integralSystem(i, i) = 2;
					secondDifferences(i) = 3 * (values(i) - values(i - 1));
				}
				else
				{
					integralSystem(i, 0) = 1;
					integralSystem(i, i - 1) = 1;
					integralSystem(i, i) = 4;
					secondDifferences(i) = 3 * (values(i) - values(i - 1));
				}
			}
			else
			{
				integralSystem(i, i - 1) = 1;
				integralSystem(i, i) = 4;
				integralSystem(i, i + 1) = 1;
				secondDifferences(i) = 3 * (values(i + 1) - values(i - 1));
			}
		}

		VectorXf secondDivs = integralSystem.llt().solve(secondDifferences);

		int tIndex = 0;
		if (closed) 
		{
			tIndex = (int)t % n;
		}
		else
		{
			tIndex = (int)min(max(t, 0.f), n - 1.f);
		}
		int tIndexNext = (tIndex + 1) % n;

		float tAdjusted = t - tIndex;

		float a = values[tIndex];
		float b = secondDivs[tIndex];
		float c = 3 * (values[tIndexNext] - values[tIndex]) - 2 * secondDivs[tIndex] - secondDivs[tIndexNext];
		float d = 2 * (values[tIndex] - values[tIndexNext]) + secondDivs[tIndex] + secondDivs[tIndexNext];

		return a + b * tAdjusted + c * pow(tAdjusted, 2) + d * pow(tAdjusted, 3);
	}

	VectorXf CubicNatural(const MatrixXf &values, float t, bool closed)
	{
		const unsigned int valueCount = values.cols();
		const unsigned int pieceCount = values.rows();

		VectorXf interpolated(valueCount);
		for (unsigned valueIndex = 0; valueIndex < valueCount; valueIndex++)
		{
			interpolated(valueIndex) = CubicNaturalScalar(values.col(valueIndex), t, closed);
		}

		return interpolated;
	}

	VectorXf CubicNaturalOpened(const MatrixXf &values, float t)
	{
		return CubicNatural(values, t, false);
	}

	VectorXf CubicNaturalClosed(const MatrixXf &values, float t)
	{
		return CubicNatural(values, t, true);
	}
};