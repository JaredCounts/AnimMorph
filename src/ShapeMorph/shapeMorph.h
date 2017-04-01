#pragma once
#include "../Mesh2D/mesh2D.h"

#include <map>

class ShapeMorph
{
public:

	static Mesh2D Interpolate(const Mesh2D& start, const Mesh2D& end, float t);

private:

	typedef std::pair<unsigned int, unsigned int> IntPair;

	typedef std::map< IntPair, unsigned int> IntPairToInt;

	static void FlattenEdgeLengths(
		VectorXf &edgeLengths,
		const VectorXf &edgeLengthsOriginal,
		const Matrix2Xi &edges,
		const Matrix3Xi &triangles,
		const unsigned int pointCount,
		const IntPairToInt &edgeToEdgeIndex);

	static void EmbedMesh(
		Matrix2Xf &points,
		const VectorXf &edgeLengths,
		const Matrix3Xi &triangles,
		const IntPairToInt &edgeToEdgeIndex);

};