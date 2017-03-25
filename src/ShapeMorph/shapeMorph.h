#pragma once
#include "../Mesh2D/mesh2D.h"

class ShapeMorph
{
public:

	static Mesh2D Interpolate(const Mesh2D& start, const Mesh2D& end, float t);

private:
	
	void FlattenEdgeLengths(
		VectorXf &edgeLengths,
		const VectorXf &edgeLengthsOriginal,
		const Matrix2Xi &edges,
		const Matrix3Xi &triangles,
		const unsigned int pointCount);

	void EmbedMesh(MatrixXf &points, 
					const VectorXf &edgeLengths, 
					const const VectorXi edges, 
					const Matrix3Xi triangles);

};