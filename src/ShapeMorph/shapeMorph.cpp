#include "shapeMorph.h"

Mesh2D ShapeMorph::Interpolate(const Mesh2D &start, const Mesh2D &end, float t)
{
	// matrices must be topologically the same
	assert((start.GetTriangles() - end.GetTriangles()).norm() == 0);

	// 1. generate new edge lengths
	const Matrix2Xf startPoints = start.GetPoints_Local();
	const Matrix2Xf endPoints = end.GetPoints_Local();

	// topology is same, so the edge indices should be identical between both meshes
	const Matrix2Xi edges = start.GetEdges();

	unsigned int edgeCount = edges.cols();

	VectorXf interpEdgeLengths(edgeCount);

	for (unsigned int i = 0; i < edgeCount; i++)
	{
		const unsigned int vertAIndex = edges(0, i);
		const unsigned int vertBIndex = edges(1, i);
		float startEdgeLengthSq = (startPoints.col(vertAIndex) - startPoints.col(vertBIndex)).squaredNorm();
		float endEdgeLengthSq = (endPoints.col(vertAIndex) - endPoints.col(vertBIndex)).squaredNorm();
	
		interpEdgeLengths(i) = sqrt((1 - t) * startEdgeLengthSq + t * endEdgeLengthSq);
	}

	// 2. flatten mesh using Conformal Equivalence of Triangle Mesh

	// 3. Embed mesh into 2D Euclidean space

	return Mesh2D();
}

void ShapeMorph::FlattenEdges(VectorXf &edgeLengths, const VectorXi edges)
{
	// Newton step:

		// 1. Generate energy

		// 2. Generate Hessian

		// 3. Update "u" vector 
			// xxx: side remark, what do we call "u"?
			// they're the exponential edge contributions 
			// at each vector to the flattened edge
			// i.e. flatEdgeLength = e^(u[i] + u[j]) * edgeLength
			// so u = "vertexLogEdgeContribution"
}

void ShapeMorph::EmbedMesh(MatrixXf & points, const VectorXf & edgeLengths, const const VectorXi edges, const Matrix3Xi triangles)
{
	// 1. choose a point and direction

	// 2. while set of neighboring triangles is not empty:

		// embed neighboring triangle

}