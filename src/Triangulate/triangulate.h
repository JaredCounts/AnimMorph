#pragma once
#include "../linearAlgebra.h"
#include "../Mesh2D/mesh2D.h"

namespace Triangulate
{
	Mesh2D Triangulate(
		const Matrix2Xi &edges,
		const Matrix2Xf &vertices);
};
