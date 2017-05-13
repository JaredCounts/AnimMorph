#pragma once
#include "../Mesh2D/mesh2D.h"
#include "../Mesh2D/meshHelper.h"
#include "../linearAlgebra.h"
#include <Eigen/Sparse>

namespace Laplacian
{
	SparseMatrix<float> CotLaplacian(
		const Matrix2Xf &vertices,
		const Matrix3Xi &triangles,
		const MeshHelper &meshHelper
	);

	SparseMatrix<float> CotLaplacian(
		const Matrix3Xf &triangleAngles,
		const Matrix3Xi &triangles,
		const MeshHelper &meshHelper,
		const std::unordered_map<unsigned int, int> &vertIndexToCoeffIndex,
		const unsigned int coeffCount);
};