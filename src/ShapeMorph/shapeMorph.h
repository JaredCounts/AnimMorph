#pragma once
#include "../Mesh2D/mesh2D.h"

class ShapeMorph
{
public:

	static Mesh2D Interpolate(const Mesh2D& start, const Mesh2D& end, float t);

private:
	
	void FlattenEdges(VectorXf &edgeLengths, const VectorXi edges);

	void EmbedMesh(MatrixXf &points, 
					const VectorXf &edgeLengths, 
					const const VectorXi edges, 
					const Matrix3Xi triangles);

};