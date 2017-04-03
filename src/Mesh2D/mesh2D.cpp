#include "mesh2D.h"

#include <set>
#include <assert.h>

Mesh2D::Mesh2D(const Mesh2D & mesh) :
	points(mesh.GetPoints_Local()),
	triangles(mesh.GetTriangles()),
	transform(mesh.GetTransform())
{
}

Mesh2D::Mesh2D(Matrix2Xf points, Matrix3Xi triangles) :
	points(points),
	triangles(triangles),
	transform(Translation2f(0,0))
{
}

Mesh2D::Mesh2D() :
	points(),
	triangles(),
	transform(Translation2f(0,0))
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

const Matrix2Xf Mesh2D::GetPoints_World() const
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
const Matrix2Xi Mesh2D::GetEdges() const
{	
	// XXX: could be more efficient.
	// XXX: could also cache the results 

	std::set<std::pair<unsigned int, unsigned int>> edgeSet;

	for (unsigned int i = 0; i < TriangleCount(); i++)
	{
		const Vector3i triangle = triangles.col(i);
		for (unsigned int j = 0; j < 3; j++)
		{
			std::pair<int, int> edge(triangle[j], triangle[(j + 1) % 3]);
			std::pair<int, int> edgeReversed(edge.second, edge.first);

			if (edgeSet.find(edge) == edgeSet.end() 
				&& edgeSet.find(edgeReversed) == edgeSet.end())
			{
				edgeSet.emplace(edge);
			}
		}
	}

	Matrix2Xi edges;
	edges.conservativeResize(2, edgeSet.size());
	unsigned int i = 0;
	for (auto &edge : edgeSet)
	{
		edges.col(i) = Vector2i(edge.first, edge.second);
		i += 1;
	}

	return edges;
}

const Transform<float, 2, TransformTraits::Affine>& Mesh2D::GetTransform() const
{
	return transform;
}

