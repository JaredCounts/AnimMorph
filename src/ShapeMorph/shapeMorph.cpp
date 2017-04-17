#include "shapeMorph.h"

#include <iostream>

#include "ShapeMorphImpl/edgeInterpolate.h"
#include "ShapeMorphImpl/flattenEdges.h"
#include "ShapeMorphImpl/embedMesh.h"

namespace ShapeMorph
{

	P_Mesh2D Interpolate(
		const P_Mesh2Ds &meshes,
		float t,
		const MeshHelper &meshHelper,
		const Interpolation::InterpolationFunc &interpolateFunction)
	{
		const unsigned int meshCount = meshes.size();

		assert(meshCount > 0);

		// t must be between 0 and mesh count
		assert(0 <= t && t <= meshCount);

		const P_Mesh2D &startMesh = meshes[0];

		// topology is same, so the edge indices should be identical between both meshes
		const Matrix3Xi &triangles = startMesh->GetTriangles();
		const Matrix2Xi &edges = meshHelper.GetEdges();

		const unsigned int edgeCount = edges.cols();
		const unsigned int pointCount = startMesh->PointCount();

		// matrices must be topologically the same
		for (auto &mesh : meshes)
		{
			assert(mesh->GetTriangles() == triangles);
		}

		// 1. generate new edge lengths
		// std::cout << "1. generating new edge lengths.\n";

		VectorXf interpEdgeLengths(edgeCount);
		ShapeMorphImpl::InterpolateEdgeLengths(
			interpEdgeLengths, 
			edges,
			meshes, 
			t, 
			interpolateFunction);

		const VectorXf edgeLengthsCopy(interpEdgeLengths);

		// 2. flatten mesh using Conformal Equivalence of Triangle Mesh
		// std::cout << "2. flattening mesh.\n";
		ShapeMorphImpl::FlattenEdgeLengths(
			interpEdgeLengths,
			edges, 
			triangles, 
			pointCount, 
			meshHelper);

		// 3. Embed mesh into 2D Euclidean space
		// std::cout << "3. embedding mesh.\n";
		Matrix2Xf points = Matrix2Xf::Zero(2, pointCount);
		ShapeMorphImpl::EmbedMesh(
			points, 
			interpEdgeLengths, 
			triangles, 
			edges, 
			meshHelper);

		return P_Mesh2D(new Mesh2D(points, triangles));
	}
}
