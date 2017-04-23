#include "outlineToMesh2D.h"

#include "../Triangulate/triangulate.h"

#include <map>

bool vecCompare(Vector2f v, Vector2f w) 
{
	for (size_t i = 0; i<v.size(); ++i)
	{
		if (v[i]<w[i]) return true;
		if (v[i]>w[i]) return false;
	}
	return false;
}

Mesh2D OutlineToMesh2D::OutlineToMesh2D(const Matrix2Xf &points)
{	
	// 1. Convert face to triangles
	Matrix2Xf result;
	if (!Triangulate::Triangulate::Process(points, result))
	{
		assert(false);
		throw "Could not triangulate ... maybe this shape has holes or crossing edges?";
	}
	int triangleCount = result.cols() / 3;

	// 2. Get rid of duplicate points in triangles
	bool(*fn_pt)(Vector2f, Vector2f) = vecCompare;
	std::map<Vector2f, int, 
		bool(*)(Vector2f, Vector2f)> 
		pointToIndex(
			[](Vector2f lhs, Vector2f rhs) { 
				for (size_t i = 0; i < 2; i++)
				{
					if (lhs[i]<rhs[i]) return true;
					if (lhs[i]>rhs[i]) return false;
				}
				return false;
			});


	Matrix2Xf meshPoints;
	Vector2f average(0,0);

	int currentIndex = 0;
	for (int colIndex = 0; colIndex < result.cols(); colIndex++)
	{
		const Vector2f &point = result.col(colIndex);

		if (pointToIndex.find(point) == pointToIndex.end())
		{
			pointToIndex[point] = currentIndex;

			meshPoints.conservativeResize(meshPoints.rows(), currentIndex+1);
			meshPoints.col(currentIndex) = Vector2f(point);
			
			average += point;

			currentIndex++;
		}

	}

	average /= meshPoints.cols();

	Matrix3Xi triangles(3, triangleCount);

	for (int triIndex = 0; triIndex < triangleCount; triIndex++)
	{
		int pointIndexA = pointToIndex[result.col(triIndex * 3 + 0)];
		int pointIndexB = pointToIndex[result.col(triIndex * 3 + 1)];
		int pointIndexC = pointToIndex[result.col(triIndex * 3 + 2)];
		triangles.col(triIndex) = Vector3i(pointIndexA, pointIndexB, pointIndexC);
	}

	// center points
	meshPoints = Transform<float, 2, TransformTraits::Affine>(Translation2f(-average)) * meshPoints;

	meshPoints *= 0.1;

	// 3. Return our Mesh2D
	return Mesh2D(meshPoints, triangles);
}
