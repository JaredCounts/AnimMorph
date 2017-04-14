#pragma once
#include "../Mesh2D/mesh2D.h"

#include <map>

#include "../Mesh2D/meshHelper.h"

class ShapeMorph
{
public:
	typedef std::function<VectorXf(const MatrixXf&, float)> InterpolateFunction;


	static P_Mesh2D Interpolate(
		const P_Mesh2Ds &meshes,
		float t,
		const MeshHelper &meshHelper,
		const InterpolateFunction &interpolateFunction);

private:
	typedef std::pair<unsigned int, unsigned int> UIntPair;

	static void FlattenEdgeLengths(
		VectorXf &edgeLengths,
		const VectorXf &edgeLengthsOriginal,
		const Matrix2Xi &edges,
		const Matrix3Xi &triangles,
		const unsigned int pointCount,
		const MeshHelper &meshHelper);

	static void EmbedMesh(
		Matrix2Xf &points,
		const VectorXf &edgeLengths,
		const Matrix3Xi &triangles,
		const Matrix2Xi &edges,
		const MeshHelper &meshHelper);
};