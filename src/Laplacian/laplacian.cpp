#include "laplacian.h"

namespace Laplacian
{
	typedef std::pair<unsigned int, unsigned int> UIntPair;

	float Cot(const float &angle)
	{
		return tan(M_PI / 2.0 - angle);
	}

	/*
	Given edge length opposite of vertex A,
	and edge lengths incident to vertex A
	Returns the angle at vertex A between the two incident edges
	*/
	float VertexAngleInTriangle(
		const float &oppEdgeLength,
		const float &adjEdgeLengthA,
		const float &adjEdgeLengthB)
	{

		// triangle inequality
		assert(oppEdgeLength <= adjEdgeLengthA + adjEdgeLengthB);

		assert(oppEdgeLength > 0 && std::isfinite(oppEdgeLength));
		assert(adjEdgeLengthA > 0 && std::isfinite(adjEdgeLengthA));
		assert(adjEdgeLengthB > 0 && std::isfinite(adjEdgeLengthB));

		// using cosine rule
		float angle = acos((Sq(adjEdgeLengthA) + Sq(adjEdgeLengthB) - Sq(oppEdgeLength)) / (2 * adjEdgeLengthA * adjEdgeLengthB));

		assert(!std::isnan(angle));

		return angle;
	}

	SparseMatrix<float> CotLaplacian(
		const Matrix2Xf & vertices,
		const Matrix3Xi & triangles, 
		const MeshHelper & meshHelper)
	{
		const Matrix2Xi &edges = meshHelper.GetEdges();

		unsigned int edgeCount = edges.cols();
		unsigned int triangleCount = triangles.cols();
		unsigned int vertexCount = vertices.cols();

		VectorXf edgeLengths(edgeCount);

		for (unsigned int edgeIndex = 0; edgeIndex < edges.cols(); edgeIndex++)
		{
			const Vector2f &vertexA = vertices.col(edges(0, edgeIndex));
			const Vector2f &vertexB = vertices.col(edges(1, edgeIndex));
			edgeLengths[edgeIndex] = (vertexA - vertexB).norm();
		}

		unsigned int coeffCount = 0;
		// a coeff index of -1 means the vertex is a boundary vertex
		std::unordered_map<unsigned int, int> vertIndexToCoeffIndex;

		for (unsigned int vertIndex = 0; vertIndex < vertexCount; vertIndex++)
		{
			vertIndexToCoeffIndex[vertIndex] = coeffCount;
			coeffCount++;
		}


		Matrix3Xf triangleAngles = Matrix3Xf::Zero(3, triangleCount);

		for (unsigned int triIndex = 0; triIndex < triangleCount; triIndex++)
		{
			const Vector3i &triangle = triangles.col(triIndex);

			unsigned int vertIndexA = triangle[0];
			unsigned int vertIndexB = triangle[1];
			unsigned int vertIndexC = triangle[2];

			// pay special care to the fact that edgeA is the edge OPPOSITE of vertex A
			// that is, edgeA does not include vertex A.
			const unsigned int &edgeIndexA = meshHelper.EdgeToEdgeIndex(triangle[1], triangle[2]);
			const unsigned int &edgeIndexB = meshHelper.EdgeToEdgeIndex(triangle[0], triangle[2]);
			const unsigned int &edgeIndexC = meshHelper.EdgeToEdgeIndex(triangle[0], triangle[1]);

			assert(edgeIndexA < edges.size() && edgeIndexB < edges.size() && edgeIndexC < edges.size());

			const float &edgeLengthA = edgeLengths[edgeIndexA];
			const float &edgeLengthB = edgeLengths[edgeIndexB];
			const float &edgeLengthC = edgeLengths[edgeIndexC];

			// check for special cases of triangle inequality being violated
			if (edgeLengthA > edgeLengthB + edgeLengthC)
			{
				triangleAngles(0, triIndex) = M_PI;
				triangleAngles(1, triIndex) = 0;
				triangleAngles(2, triIndex) = 0;
			}
			else if (edgeLengthB > edgeLengthA + edgeLengthC)
			{
				triangleAngles(0, triIndex) = 0;
				triangleAngles(1, triIndex) = M_PI;
				triangleAngles(2, triIndex) = 0;
			}
			else if (edgeLengthC > edgeLengthA + edgeLengthB)
			{
				triangleAngles(0, triIndex) = 0;
				triangleAngles(1, triIndex) = 0;
				triangleAngles(2, triIndex) = M_PI;
			}
			else
			{
				triangleAngles(0, triIndex) = VertexAngleInTriangle(edgeLengthA, edgeLengthB, edgeLengthC);
				triangleAngles(1, triIndex) = VertexAngleInTriangle(edgeLengthB, edgeLengthA, edgeLengthC);
				triangleAngles(2, triIndex) = M_PI - (triangleAngles(0, triIndex) + triangleAngles(1, triIndex));
			}
		}

		return CotLaplacian(
			triangleAngles,
			triangles,
			meshHelper,
			vertIndexToCoeffIndex,
			coeffCount);
	}

	SparseMatrix<float> CotLaplacian(
		const Matrix3Xf &triangleAngles,
		const Matrix3Xi &triangles,
		const MeshHelper &meshHelper,
		const std::unordered_map<unsigned int, int> &vertIndexToCoeffIndex,
		const unsigned int coeffCount)
	{
		const unsigned int triangleCount = triangles.cols();

		// first we build a map of edge pairs to laplacian coefficients
		// then convert those to triplets
		// then load those into the sparse matrix (acc. to Eigen, this is the best way forward)

		std::unordered_map<UIntPair, float, pairhash> laplacianCoeffs;

		// generate coeffs
		for (unsigned int triIndex = 0; triIndex < triangleCount; triIndex++)
		{
			const Vector3i &triangle = triangles.col(triIndex);

			const unsigned int &vertIndexA = triangle[0];
			const unsigned int &vertIndexB = triangle[1];
			const unsigned int &vertIndexC = triangle[2];

			assert(vertIndexToCoeffIndex.find(vertIndexA) != vertIndexToCoeffIndex.end());
			assert(vertIndexToCoeffIndex.find(vertIndexB) != vertIndexToCoeffIndex.end());
			assert(vertIndexToCoeffIndex.find(vertIndexC) != vertIndexToCoeffIndex.end());

			const unsigned int &coeffIndexA = vertIndexToCoeffIndex.at(vertIndexA);
			const unsigned int &coeffIndexB = vertIndexToCoeffIndex.at(vertIndexB);
			const unsigned int &coeffIndexC = vertIndexToCoeffIndex.at(vertIndexC);

			const float &triangleAngleA = triangleAngles(0, triIndex);
			const float &triangleAngleB = triangleAngles(1, triIndex);
			const float &triangleAngleC = triangleAngles(2, triIndex);

			float coeffAFactor = 0.5, coeffBFactor = 0.5, coeffCFactor = 0.5;
			{
				if (meshHelper.IsVertexOnBoundary(vertIndexA) && meshHelper.IsVertexOnBoundary(vertIndexB))
				{
					coeffAFactor = 1;
					coeffBFactor = 1;
				}
				if (meshHelper.IsVertexOnBoundary(vertIndexA) && meshHelper.IsVertexOnBoundary(vertIndexC))
				{
					coeffAFactor = 1;
					coeffCFactor = 1;
				}
				if (meshHelper.IsVertexOnBoundary(vertIndexC) && meshHelper.IsVertexOnBoundary(vertIndexB))
				{
					coeffCFactor = 1;
					coeffBFactor = 1;
				}
			}

			const float coeffA = coeffAFactor * Cot(triangleAngleA);
			const float coeffB = coeffBFactor * Cot(triangleAngleB);
			const float coeffC = coeffCFactor * Cot(triangleAngleC);

			if (coeffIndexA != -1 && coeffIndexB != -1)
			{
				laplacianCoeffs[UIntPair(coeffIndexA, coeffIndexB)] += -coeffC;
				laplacianCoeffs[UIntPair(coeffIndexB, coeffIndexA)] += -coeffC;
			}

			if (coeffIndexB != -1 && coeffIndexC != -1)
			{
				laplacianCoeffs[UIntPair(coeffIndexB, coeffIndexC)] += -coeffA;
				laplacianCoeffs[UIntPair(coeffIndexC, coeffIndexB)] += -coeffA;
			}

			if (coeffIndexA != -1 && coeffIndexC != -1)
			{
				laplacianCoeffs[UIntPair(coeffIndexA, coeffIndexC)] += -coeffB;
				laplacianCoeffs[UIntPair(coeffIndexC, coeffIndexA)] += -coeffB;
			}

			if (coeffIndexA != -1)
			{   // xxx: does this make sense?
				//      i.e. if vertex B isn't a coeff, 
				//           should it still contribute to the diagonal here?
				laplacianCoeffs[UIntPair(coeffIndexA, coeffIndexA)] += coeffB + coeffC;
			}
			if (coeffIndexB != -1)
			{
				laplacianCoeffs[UIntPair(coeffIndexB, coeffIndexB)] += coeffA + coeffC;
			}
			if (coeffIndexC != -1)
			{
				laplacianCoeffs[UIntPair(coeffIndexC, coeffIndexC)] += coeffA + coeffB;
			}
		}

		typedef std::vector<Triplet<float>> V_Triplets;

		V_Triplets laplacianEntries;

		for (auto &lapEntry : laplacianCoeffs)
		{
			const UIntPair &matrixPosition = lapEntry.first;
			const float &matrixCoeff = lapEntry.second;

			assert(!std::isnan(matrixCoeff) && std::isfinite(matrixCoeff));

			laplacianEntries.push_back(Triplet<float>(matrixPosition.first, matrixPosition.second, matrixCoeff));
		}

		SparseMatrix<float> laplacian(coeffCount, coeffCount);

		laplacian.setFromTriplets(laplacianEntries.begin(), laplacianEntries.end());

		return laplacian;
	}
}