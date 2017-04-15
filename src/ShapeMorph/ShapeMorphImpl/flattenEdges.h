#include "../../linearAlgebra.h"
#include "../../Mesh2D/meshHelper.h"

namespace ShapeMorphImpl
{

void FlattenEdgeLengths(
	VectorXf &edgeLengths,
	const Matrix2Xi &edges,
	const Matrix3Xi &triangles,
	const unsigned int pointCount,
	const MeshHelper &meshHelper);

};