#include "edgeInterpolate.h"

namespace ShapeMorphImpl
{

	void InterpolateEdgeLengths(
		VectorXf &interpEdgeLengths,
		const P_Mesh2Ds &meshes,
		float t,
		const Interpolation::InterpolationFunc &interpolateFunction)
	{
		// topology is same, so the edge indices should be identical between both meshes
		const Matrix2Xi &edges = meshes[0]->GetEdges();

		const unsigned int edgeCount = edges.cols();

		assert(interpEdgeLengths.size() == edgeCount);

		// row: edge index
		// column: interpolation index
		MatrixXf edgeLengthsSq(meshes.size(), edgeCount);

		for (unsigned int meshIndex = 0; meshIndex < meshes.size(); meshIndex++)
		{
			const P_Mesh2D &mesh = meshes[meshIndex];
			const Matrix2Xf &points = mesh->GetPoints_Local();

			for (unsigned int edgeIndex = 0; edgeIndex < edgeCount; edgeIndex++)
			{

				const unsigned int vertAIndex = edges(0, edgeIndex);
				const unsigned int vertBIndex = edges(1, edgeIndex);

				float edgeLengthSq = (points.col(vertAIndex) - points.col(vertBIndex)).squaredNorm();

				assert(edgeLengthSq != 0);

				edgeLengthsSq(meshIndex, edgeIndex) = edgeLengthSq;
			}
		}

		interpEdgeLengths = interpolateFunction(edgeLengthsSq, t);
		assert(interpEdgeLengths.size() == edgeCount);

		for (unsigned int i = 0; i < edgeCount; i++)
		{
			interpEdgeLengths[i] = sqrt(interpEdgeLengths[i]);

			assert(interpEdgeLengths[i] != 0);
			assert(!std::isnan(interpEdgeLengths[i]));
			assert(std::isfinite(interpEdgeLengths[i]));
		}
	}

}