#pragma once

#include "../Mesh2D/meshHelper.h"

namespace Geodesic
{
	VectorXf DistanceFrom(
		int vertIndex,
		const Matrix2Xf & vertices,
		const Matrix3Xi & triangles);
}