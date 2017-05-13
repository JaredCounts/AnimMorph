#include "../../linearAlgebra.h"
#include "../../Mesh2D/meshHelper.h"

namespace ShapeMorphImpl
{
	void EmbedMesh(
		Matrix2Xf &points,
		const VectorXf &edgeLengths,
		const Matrix3Xi &triangles,
		const Matrix2Xi &edges,
		const MeshHelper &meshHelper,
		float firstTriangleAngle);
}