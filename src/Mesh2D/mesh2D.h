#pragma once

#include <memory>
#include <vector>

#include "../linearAlgebra.h"

class Mesh2D
{
public:
	Mesh2D(const Mesh2D &mesh);

	Mesh2D(Matrix2Xf points, Matrix3Xi triangles);

	Mesh2D();

	void AddPoint(const Vector2f point);

	void AddTriangle(const Vector3i indices);

	void SetPoint(const unsigned int index, const Vector2f point);
	
	void Translate(const Vector2f &offset);

	int PointCount() const;
	int TriangleCount() const;

	Matrix2Xf GetPoints_World() const;
	const Matrix2Xf &GetPoints_Local() const;
	const Matrix3Xi &GetTriangles() const;

	const Transform2f &GetTransform() const;

	Vector3f color;

private:
	// geometry
	// each column contains 2 coordinate values
	Matrix2Xf points;

	// topology
	// each column contains 3 indices
	Matrix3Xi triangles; // 3 rows, X columns

	// basis
	Transform2f transform;
};

typedef std::shared_ptr<Mesh2D> P_Mesh2D;
typedef std::vector<P_Mesh2D> P_Mesh2Ds;