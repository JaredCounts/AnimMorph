#include "flattenEdges.h"

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "../../Laplacian/laplacian.h"

#include <iostream>

namespace ShapeMorphImpl
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

	void FlattenEdgeLengths(
		VectorXf &edgeLengths,
		const Matrix2Xi &edges,
		const Matrix3Xi &triangles,
		const unsigned int pointCount,
		const MeshHelper &meshHelper)
	{
		// xxx: do we need to place special consideration in boundary edges?
		//		^ yes - need to exclude boundary vertices from equations - and use 0 in place of their contributions
		// xxx: one can do solver.analyzePattern to speed up factorization btwn iterations
		//      since topology (and therefore the matrix) doesn't change
		const unsigned int edgeCount = edgeLengths.size();
		assert(edgeCount == edges.cols());

		const unsigned int triangleCount = triangles.cols();

		// std::cout << "Generating coeff map.\n";
		// our system of equations only consists of vertices not on the boundary
		// (since the boundary vertices have their edgeContrib log values = 0
		// so we need a mapping from vert index to an index to and equation coeff
		unsigned int coeffCount = 0;
		// a coeff index of -1 means the vertex is a boundary vertex
		std::unordered_map<unsigned int, int> vertIndexToCoeffIndex;
		std::unordered_map<unsigned int, unsigned int> coeffIndexToVertIndex;

		for (unsigned int vertIndex = 0; vertIndex < pointCount; vertIndex++)
		{
			if (!meshHelper.IsVertexOnBoundary(vertIndex))
			{
				vertIndexToCoeffIndex[vertIndex] = coeffCount;
				coeffIndexToVertIndex[coeffCount] = vertIndex;
				coeffCount++;
			}
			else
			{
				vertIndexToCoeffIndex[vertIndex] = -1;
			}
		}


		// this is the "u" vector found in the CETM and BDMORPH papers
		// i.e. each coefficient is some logmarithmic contribution 
		//      to the lengths of edges incident to the given vertex
		// xxx: what would be a good first "guess" here?
		VectorXf vertexEdgeContribs = VectorXf::Ones(pointCount);

		int maxIterationCount = 100;

		// Newton step:
		for (int iteration = 0; iteration < maxIterationCount; iteration++)
		{
			// 1. Generate Energy Gradient
			// std::cout << "\t1. Generate energy gradient.\n";
			VectorXf energyGradient = VectorXf::Zero(coeffCount);

			VectorXf vertexAngles = VectorXf::Zero(pointCount);
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

				vertexAngles[vertIndexA] += triangleAngles(0, triIndex);
				vertexAngles[vertIndexB] += triangleAngles(1, triIndex);
				vertexAngles[vertIndexC] += triangleAngles(2, triIndex);
			}

			assert(!vertexAngles.hasNaN());

			for (unsigned int coeffIndex = 0; coeffIndex < coeffCount; coeffIndex++)
			{
				// the gaussian curvature at a vertex may considered 
				// to be 2PI minus the sum of interior angles of incident triangles

				// since we want to "flatten" the mesh, the desired curvature should be 0
				// so our desiredAngleSum at each vertex is then 2PI
				// (for boundary vertices, it would be PI - but they are not included in our system of equations)

				// 0.5 * (desiredAngleSum - angleSum)
				const unsigned int &vertIndex = coeffIndexToVertIndex[coeffIndex];

				energyGradient[coeffIndex] = 0.5 * (2 * M_PI - vertexAngles[vertIndex]);
				assert(!std::isnan(energyGradient[coeffIndex]) && std::isfinite(energyGradient[coeffIndex]));
			}

			assert(!energyGradient.hasNaN());
			// std::cout << "\tEnergy squared norm: " << energyGradient.squaredNorm() << "\n";
			if (energyGradient.squaredNorm() < Sq(0.00001))
			{	// stopping condition
				// std::cout << "\ttook " << iteration << " iterations\n.";
				return;
			}

			// 2. Generate Energy Hessian
			// std::cout << "\t2. Generate energy hessian.\n";
			SparseMatrix<float> energyHessian = 
				0.5 *
				Laplacian::CotLaplacian(
					triangleAngles,
					triangles,
					meshHelper,
					vertIndexToCoeffIndex,
					coeffCount); //(coeffCount, coeffCount);

			// 3. Update "u" vector 
			// xxx: side remark, what do we call "u"?
			// they're the exponential edge contributions 
			// at each vector to the flattened edge
			// i.e. flatEdgeLength = e^(u[i] + u[j]) * edgeLength
			// so u = "vertexLogEdgeContribution"
			//std::cout << "\t3. Update log edge contributions.\n";
			//SparseQR<SparseMatrix<float>, COLAMDOrdering<int>> solver;
			SimplicialLDLT<SparseMatrix<float>> solver;

			// solver.analyzePattern(A);
			// solver.factorize(energyHessian);

			// std::cout << "Energy Hessian\n==============\n" << energyHessian << "\n\n";
			// std::cout << "Energy Gradient\n===============\n[" << energyGradient.norm() << "]\n" << energyGradient << "\n\n";

			// std::cout << "compute\n";
			solver.compute(energyHessian);
			assert(solver.info() == Success);

			//std::cout << "solve\n";
			VectorXf deltaEdgeContrib = solver.solve(-1 * energyGradient);
			assert(solver.info() == Success);

			// center the deltaEdgeContrib values since the problem is scale invariant
			// xxx - this isn't needed, apparently?
			//std::cout << "center\n";
			// float average = deltaEdgeContrib.sum() / deltaEdgeContrib.size();
			// deltaEdgeContrib = deltaEdgeContrib - average * VectorXf::Ones(coeffCount);

			//std::cout << "Delta Edge\n==========\n" << deltaEdgeContrib << "\n\n";

			for (unsigned int edgeIndex = 0; edgeIndex < edgeCount; edgeIndex++)
			{
				const unsigned int &vertIndexA = edges(0, edgeIndex);
				const unsigned int &vertIndexB = edges(1, edgeIndex);

				const unsigned int &coeffIndexA = vertIndexToCoeffIndex[vertIndexA];
				const unsigned int &coeffIndexB = vertIndexToCoeffIndex[vertIndexB];

				float deltaEdgeContribA = 0;
				float deltaEdgeContribB = 0;
				if (coeffIndexA != -1)
				{
					deltaEdgeContribA = deltaEdgeContrib[coeffIndexA];
				}
				if (coeffIndexB != -1)
				{
					deltaEdgeContribB = deltaEdgeContrib[coeffIndexB];
				}

				edgeLengths[edgeIndex] = edgeLengths[edgeIndex] * exp((deltaEdgeContribA + deltaEdgeContribB) / 2.f);

				assert(edgeLengths[edgeIndex] != 0 && std::isfinite(edgeLengths[edgeIndex]));
			}

			//if (deltaEdgeContrib.squaredNorm() < 0.00001)
			//{
			//	// std::cout << "\ttook " << (iteration + 1) << " iterations\n.";
			//	// std::cout << "Energy Gradient\n===============\n[" << energyGradient.norm() << "]\n" << energyGradient << "\n\n";
			//	return;
			//}
		}

		// xxx: failure!!
		assert(false);
	}
}