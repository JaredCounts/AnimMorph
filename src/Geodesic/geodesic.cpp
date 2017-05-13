#include "geodesic.h"

#include <Eigen/SparseCholesky>

#include <iostream>

#include "geodesic_algorithm_exact.h"

VectorXf Geodesic::DistanceFrom(
	int vertIndex,
	const Matrix2Xf & vertices,
	const Matrix3Xi & triangles)
{
	unsigned vertexCount = vertices.cols();
	unsigned triangleCount = triangles.cols();

	std::vector<double> points(vertexCount * 3);
	std::vector<unsigned> faces(triangleCount * 3);

	for (unsigned vertIndex = 0; vertIndex < vertexCount; vertIndex++)
	{
		const Vector2f &vertex = vertices.col(vertIndex);
		points[vertIndex * 3] = vertex[0];
		points[vertIndex * 3 + 1] = vertex[1];
		points[vertIndex * 3 + 2] = 0;
	}

	for (unsigned triIndex = 0; triIndex < triangleCount; triIndex++)
	{
		const Vector3i &triangle = triangles.col(triIndex);

		faces[triIndex * 3] = triangle[0];
		faces[triIndex * 3 + 1] = triangle[1];
		faces[triIndex * 3 + 2] = triangle[2];
	}

	geodesic::Mesh mesh;
	mesh.initialize_mesh_data(points, faces);		//create internal mesh data structure including edges

	geodesic::GeodesicAlgorithmExact algorithm(&mesh);	//create exact algorithm for the mesh
	
	geodesic::SurfacePoint source(&mesh.vertices()[vertIndex]);
	std::vector<geodesic::SurfacePoint> all_sources(1, source);

	algorithm.propagate(all_sources);	//cover the whole mesh

	VectorXf distances(vertexCount);

	for (unsigned vertIndex = 0; vertIndex < vertexCount; vertIndex++)
	{
		geodesic::SurfacePoint p(&mesh.vertices()[vertIndex]);

		double distance;
		unsigned best_source = algorithm.best_source(p, distance);

		distances[vertIndex] = distance;
	}

	return distances;
}
