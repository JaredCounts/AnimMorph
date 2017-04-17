#include "naturalCubicSpline.h"

#include <algorithm>

namespace Interpolation
{
	LLT<MatrixXf> GetSecondOrderSolver(float n, bool closed)
	{
		MatrixXf integralSystem
			= MatrixXf::Zero(n, n);

		for (int i = 0; i < n; i++)
		{
			if (i == 0)
			{
				if (closed)
				{
					integralSystem(i, i) = 4;
					integralSystem(i, i + 1) = 1;
					integralSystem(i, n - 1) = 1;
				}
				else
				{
					integralSystem(i, i) = 2;
					integralSystem(i, i + 1) = 1;
				}
			}
			else if (i == n - 1)
			{
				if (closed)
				{
					integralSystem(i, i - 1) = 1;
					integralSystem(i, i) = 2;
				}
				else
				{
					integralSystem(i, 0) = 1;
					integralSystem(i, i - 1) = 1;
					integralSystem(i, i) = 4;
				}
			}
			else
			{
				integralSystem(i, i - 1) = 1;
				integralSystem(i, i) = 4;
				integralSystem(i, i + 1) = 1;
			}
		}

		return integralSystem.llt();
	}

	VectorXf GetSecondOrderDifferences(const VectorXf &values, bool closed)
	{
		int n = values.size();

		VectorXf secondDifferences(n);

		for (int i = 0; i < n; i++)
		{
			if (closed)
			{
				secondDifferences(i) = 3 * (values((i + 1) % n) - values((n + i - 1) % n));
			}
			else
			{
				if (i == 0)
				{
					secondDifferences(i) = 3 * (values(i + 1) - values(i));
				}
				else if (i == n - 1)
				{
					secondDifferences(i) = 3 * (values(i) - values(i - 1));
				}
				else
				{
					secondDifferences(i) = 3 * (values(i) - values(i - 1));
				}
			}
		}

		return secondDifferences;
	}

	float CubicNaturalScalarImpl(const VectorXf &values, float t, bool closed, const LLT<MatrixXf> &solver)
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

		VectorXf secondOrderDifferences = GetSecondOrderDifferences(values, closed);
		VectorXf secondOrderValues = solver.solve(secondOrderDifferences);

		int tIndex = 0;
		if (closed)
		{
			tIndex = (int)t % n;
		}
		else
		{
			tIndex = (int)std::min(std::max(t, 0.f), n - 1.f);
		}
		int tIndexNext = (tIndex + 1) % n;

		float tAdjusted = t - tIndex;

		float a = values[tIndex];
		float b = secondOrderValues[tIndex];
		float c =
			3 * (values[tIndexNext] - values[tIndex]) - 2 * secondOrderValues[tIndex] - secondOrderValues[tIndexNext];
		float d =
			2 * (values[tIndex] - values[tIndexNext]) + secondOrderValues[tIndex] + secondOrderValues[tIndexNext];

		return a + b * tAdjusted + c * pow(tAdjusted, 2) + d * pow(tAdjusted, 3);
	}

	VectorXf CubicNatural(const MatrixXf &values, float t, bool closed)
	{
		const unsigned int valueCount = values.cols();
		const unsigned int pieceCount = values.rows();

		VectorXf interpolated(valueCount);


		auto &solver = GetSecondOrderSolver(pieceCount, closed);
		for (unsigned valueIndex = 0; valueIndex < valueCount; valueIndex++)
		{
			interpolated(valueIndex) = CubicNaturalScalarImpl(values.col(valueIndex), t, closed, solver);
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

	InterpolationFunc CubicNaturalSplineFunc(bool closed)
	{
		if (closed)
		{
			return CubicNaturalClosed;
		}
		else
		{
			return CubicNaturalOpened;
		}
	}


}