#include "mesh2D.h"

#include <assert.h>

Mesh2D::Mesh2D()
{
}

void Mesh2D::AddPoint(const Vector2f point)
{
	points.conservativeResize(points.rows(), points.cols() + 1);
	points.col(points.cols() - 1) = point;
}

//void Mesh2D::AddPoints(const V_Vector2f points)
//{
//	for (auto &point : points)
//	{
//		AddPoint(point);
//	}
//}

void Mesh2D::AddTriangle(const Vector3i indices)
{
	// triangle indices must already be in mesh
	assert(indices[0] < PointCount());
	assert(indices[1] < PointCount());
	assert(indices[2] < PointCount());

	triangles.conservativeResize(triangles.rows(), triangles.cols() + 1);
	triangles.col(triangles.cols() - 1) = indices;
}

int Mesh2D::PointCount() const
{
	return points.cols();
}

int Mesh2D::TriangleCount() const
{
	return triangles.cols();
}

const Matrix2Xf &Mesh2D::GetPoints() const
{
	return points;
}

const Matrix3Xi &Mesh2D::GetTriangles() const
{
	return triangles;
}

