#include "mesh2D.h"

#include <unordered_set>

#include <assert.h>

#include "../Misc/pairHash.h"


Mesh2D::Mesh2D(const Mesh2D & mesh) :
	points(mesh.GetPoints_Local()),
	triangles(mesh.GetTriangles()),
	transform(mesh.GetTransform()),
	color(0.0, 0.0, 0.0)
{
}

Mesh2D::Mesh2D(Matrix2Xf points, Matrix3Xi triangles) :
	points(points),
	triangles(triangles),
	transform(Translation2f(0,0)),
	color(0.0, 0.0, 0.0)
{
}

Mesh2D::Mesh2D() :
	points(),
	triangles(),
	transform(Translation2f(0,0)),
	color(0.0, 0.0, 0.0)
{
}

void Mesh2D::AddPoint(const Vector2f point)
{
	points.conservativeResize(points.rows(), points.cols() + 1);
	points.col(points.cols() - 1) = point;
}

void Mesh2D::AddTriangle(const Vector3i indices)
{
	// triangle indices must already be in mesh
	assert(indices[0] < PointCount());
	assert(indices[1] < PointCount());
	assert(indices[2] < PointCount());

	triangles.conservativeResize(triangles.rows(), triangles.cols() + 1);
	triangles.col(triangles.cols() - 1) = indices;
}

void Mesh2D::SetPoint(const unsigned int index, const Vector2f point)
{
	assert(index < PointCount());

	points.col(index) = point;
}

void Mesh2D::Translate(const Vector2f &offset)
{
	transform.translation() += offset;
}

int Mesh2D::PointCount() const
{
	return points.cols();
}

int Mesh2D::TriangleCount() const
{
	return triangles.cols();
}

Matrix2Xf Mesh2D::GetPoints_World() const
{ 
	return transform * points;
}

const Matrix2Xf &Mesh2D::GetPoints_Local() const
{
	return points;
}

const Matrix3Xi &Mesh2D::GetTriangles() const
{
	return triangles;
}

const Transform<float, 2, TransformTraits::Affine>& Mesh2D::GetTransform() const
{
	return transform;
}

