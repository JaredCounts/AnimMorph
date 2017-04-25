#include "bezier.h"

namespace Interpolation
{
	VectorXf Bezier(const MatrixXf &values, const VectorXf &controlA, const VectorXf &controlB, float t)
	{
		// following setup described in http://www.cs.cmu.edu/afs/cs/academic/class/15462-s10/www/lec-slides/lec06.pdf
		const unsigned int pieceCount = values.rows();
		const unsigned int valueCount = values.cols();

		assert(pieceCount == 2);

		assert(valueCount == controlA.size());
		assert(valueCount == controlB.size());

		// insert controlA and B between the two values
		MatrixXf controlPoints(4, valueCount);
		controlPoints.row(0) = values.row(0);
		controlPoints.row(1) = controlA.transpose();
		controlPoints.row(2) = controlB.transpose();
		controlPoints.row(3) = values.row(1);

		RowVector4f splineCoeffs;
		splineCoeffs << t * t * t, t * t, t, 1;

		Matrix4f basis;
		basis <<
			-1, 3, -3, 1,
			3, -6, 3, 0,
			-3, 3, 0, 0,
			1, 0, 0, 0;


		return (splineCoeffs * basis * controlPoints).transpose();
	}

	InterpolationFunc BezierFunc(const VectorXf &controlPointA, const VectorXf &controlPointB)
	{
		InterpolationFunc bezierFunc = [controlPointA, controlPointB](const MatrixXf &values, float t) {
			return Bezier(values, controlPointA, controlPointB, t);
		};

		return bezierFunc;
	}
}