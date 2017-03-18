#pragma once

#include "../linearAlgebra.h"

class Mesh2D
{
public:
	Mesh2D();

	void AddPoint(const Vector2f point);

	void AddTriangle(const Vector3i indices);

	int PointCount() const;

	int TriangleCount() const;

	const Matrix2Xf &GetPoints() const;

	const Matrix3Xi &GetTriangles() const;

private:
	// geometry
	// each column contains 2 coordinate values
	Matrix2Xf points;

	// topology
	// each column contains 3 indices
	Matrix3Xi triangles; // 3 rows, X columns
};