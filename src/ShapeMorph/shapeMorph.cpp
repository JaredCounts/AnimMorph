#include "shapeMorph.h"

#include <map>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

Mesh2D ShapeMorph::Interpolate(const Mesh2D &start, const Mesh2D &end, float t)
{
	// matrices must be topologically the same
	assert((start.GetTriangles() - end.GetTriangles()).norm() == 0);

	// 1. generate new edge lengths
	const Matrix2Xf startPoints = start.GetPoints_Local();
	const Matrix2Xf endPoints = end.GetPoints_Local();

	// topology is same, so the edge indices should be identical between both meshes
	const Matrix2Xi edges = start.GetEdges();

	unsigned int edgeCount = edges.cols();

	VectorXf interpEdgeLengths(edgeCount);

	for (unsigned int i = 0; i < edgeCount; i++)
	{
		const unsigned int vertAIndex = edges(0, i);
		const unsigned int vertBIndex = edges(1, i);
		float startEdgeLengthSq = (startPoints.col(vertAIndex) - startPoints.col(vertBIndex)).squaredNorm();
		float endEdgeLengthSq = (endPoints.col(vertAIndex) - endPoints.col(vertBIndex)).squaredNorm();

		interpEdgeLengths(i) = sqrt((1 - t) * startEdgeLengthSq + t * endEdgeLengthSq);
	}

	// 2. flatten mesh using Conformal Equivalence of Triangle Mesh

	// 3. Embed mesh into 2D Euclidean space

	return Mesh2D();
}

float Sq(const float &val)
{
	return val * val;
}

float VertexAngleInTriangle(
	const float &oppEdgeLength, 
	const float &adjEdgeLengthA, 
	const float &adjEdgeLengthB)
{
	/*
	Given edge length opposite of vertex A,
	and edge lengths incident to vertex A
	Returns the angle at vertex A between the two incident edges
	*/

	// using cosine rule
	return acos((Sq(adjEdgeLengthA) + Sq(adjEdgeLengthB) - Sq(oppEdgeLength)) / (2 * adjEdgeLengthA * adjEdgeLengthB));
}

float Cot(const float &angle)
{
	return tan(M_PI / 2.0 - angle);
}

void ShapeMorph::FlattenEdgeLengths(
	VectorXf &edgeLengths, 
	const VectorXf &edgeLengthsOriginal,
	const Matrix2Xi &edges, 
	const Matrix3Xi &triangles,
	const unsigned int pointCount)
{
	// xxx: do we need to place special consideration in boundary edges?
	// xxx: one can do solver.analyzePattern to speed up factorization btwn iterations
	//      since topology (and therefore the matrix) doesn't change
	const unsigned int edgeCount = edgeLengths.size();
	assert(edgeCount == edges.cols());

	const unsigned int triangleCount = triangles.cols();

	// this is the "u" vector found in the CETM and BDMORPH papers
	// i.e. each coefficient is some logmarithmic contribution 
	//      to the lengths of edges incident to the given vertex
	// xxx: what would be a good first "guess" here?
	VectorXf vertexEdgeContribs = VectorXf::Ones(pointCount);

	// we need to some how map triangles to edges so we can calculate angles
	// make a map of edge to edgeIndex
	// this will make it easy and efficient to access edges from triangles
	typedef std::pair<unsigned int, unsigned int> intPair;

	std::map<intPair, unsigned int> edgeToEdgeIndex;

	for (unsigned int edgeIndex = 0; edgeIndex < edgeCount; edgeIndex++)
	{
		unsigned int vertIndexA = edges(0, edgeIndex);
		unsigned int vertIndexB = edges(1, edgeIndex);

		intPair edge(vertIndexA, vertIndexB);
		intPair edgeReversed(vertIndexB, vertIndexA);

		edgeToEdgeIndex.emplace(edge, edgeIndex);
		edgeToEdgeIndex.emplace(edgeReversed, edgeIndex);
	}

	int maxIterationCount = 100;

	// Newton step:
	for (int iteration = 0; iteration < maxIterationCount; iteration++)
	{

		// 1. Generate Energy Gradient
		VectorXf energyGradient;

		VectorXf vertexAngles = VectorXf::Zero(pointCount);
		Matrix3Xf triangleAngles = Matrix3Xf::Zero(3, triangleCount);

		for (unsigned int triIndex = 0; triIndex < triangleCount; triIndex++)
		{
			const Vector3i &triangle = triangles.col(triIndex);

			unsigned int vertIndexA = triangle[0];
			unsigned int vertIndexB = triangle[1];
			unsigned int vertIndexC = triangle[2];

			// note, vertIndexA is opposite edgeIndexA
			// and similarly for B and C
			unsigned int edgeIndexA = edgeToEdgeIndex[intPair(triangle[1], triangle[2])];
			unsigned int edgeIndexB = edgeToEdgeIndex[intPair(triangle[0], triangle[2])];
			unsigned int edgeIndexC = edgeToEdgeIndex[intPair(triangle[0], triangle[1])];

			const float &edgeLengthA = edgeLengths[edgeIndexA];
			const float &edgeLengthB = edgeLengths[edgeIndexB];
			const float &edgeLengthC = edgeLengths[edgeIndexC];

			triangleAngles(0, triIndex) = VertexAngleInTriangle(edgeLengthA, edgeLengthB, edgeLengthC);
			triangleAngles(1, triIndex) = VertexAngleInTriangle(edgeLengthB, edgeLengthA, edgeLengthC);
			triangleAngles(2, triIndex) = M_PI - (triangleAngles(0, triIndex) + triangleAngles(1, triIndex));

			vertexAngles[vertIndexA] += triangleAngles(0, triIndex);
			vertexAngles[vertIndexB] += triangleAngles(1, triIndex);
			vertexAngles[vertIndexC] += triangleAngles(2, triIndex);
		}

		for (unsigned int vertIndex = 0; vertIndex < pointCount; vertIndex++)
		{
			// the gaussian curvature at a vertex may considered 
			// to be 2PI minus the sum of interior angles of incident triangles

			// since we want to "flatten" the mesh, the desired curvature should be 0
			// so our desiredAngleSum at each vertex is then 2PI, or PI for boundary vertices

			// 0.5 * (desiredAngleSum - angleSum)
			energyGradient[vertIndex] = 0.5 * (2 * M_PI - vertexAngles[vertIndex]);

			// XXX: boundary vertices?
		}

		if (energyGradient.squaredNorm() < 0.00001)
		{	// stopping condition
			return;
		}

		// 2. Generate Energy Hessian
		SparseMatrix<float> energyHessian(pointCount, pointCount);

		// first we build a map of edge pairs to laplacian coefficients
		// then convert those to triplets
		// then load those into the sparse matrix (acc. to Eigen, this is the best way forward)

		std::map<intPair, float> laplacianCoeffs;

		// the energy Hessian is the same as 1/2 the cot-Laplacian of the mesh
		for (unsigned int triIndex = 0; triIndex < triangleCount; triIndex++)
		{
			const Vector3i &triangle = triangles.col(triIndex);

			unsigned int vertIndexA = triangle[0];
			unsigned int vertIndexB = triangle[1];
			unsigned int vertIndexC = triangle[2];

			const float &triangleAngleA = triangleAngles(0, triIndex);
			const float &triangleAngleB = triangleAngles(1, triIndex);
			const float &triangleAngleC = triangleAngles(2, triIndex);

			const float halfCotAngleA = 0.5 * Cot(triangleAngleA);
			const float halfCotAngleB = 0.5 * Cot(triangleAngleB);
			const float halfCotAngleC = 0.5 * Cot(triangleAngleC);

			laplacianCoeffs[intPair(vertIndexA, vertIndexB)] += -halfCotAngleC;
			laplacianCoeffs[intPair(vertIndexB, vertIndexA)] += -halfCotAngleC;

			laplacianCoeffs[intPair(vertIndexB, vertIndexC)] += -halfCotAngleA;
			laplacianCoeffs[intPair(vertIndexC, vertIndexB)] += -halfCotAngleA;

			laplacianCoeffs[intPair(vertIndexA, vertIndexC)] += -halfCotAngleB;
			laplacianCoeffs[intPair(vertIndexC, vertIndexA)] += -halfCotAngleB;

			laplacianCoeffs[intPair(vertIndexA, vertIndexA)] += halfCotAngleB + halfCotAngleC;
			laplacianCoeffs[intPair(vertIndexB, vertIndexB)] += halfCotAngleA + halfCotAngleC;
			laplacianCoeffs[intPair(vertIndexC, vertIndexC)] += halfCotAngleA + halfCotAngleB;
		}

		typedef std::vector<Triplet<float>> V_Triplets;

		V_Triplets laplacianEntries;

		for (auto &lapEntry : laplacianCoeffs)
		{
			const intPair &matrixPosition = lapEntry.first;
			const float &matrixCoeff = lapEntry.second;

			laplacianEntries.push_back(Triplet<float>(matrixPosition.first, matrixPosition.second, matrixCoeff));
		}

		energyHessian.setFromTriplets(laplacianEntries.begin(), laplacianEntries.end());

		// 3. Update "u" vector 
			// xxx: side remark, what do we call "u"?
			// they're the exponential edge contributions 
			// at each vector to the flattened edge
			// i.e. flatEdgeLength = e^(u[i] + u[j]) * edgeLength
			// so u = "vertexLogEdgeContribution"
		SimplicialLDLT<SparseMatrix<float> > solver;
		// solver.analyzePattern(A);   // for this step the numerical values of A are not used
		solver.factorize(energyHessian);
		VectorXf deltaEdgeContrib = solver.solve(-1 * energyGradient);

		for (unsigned int edgeIndex = 0; edgeIndex < edgeCount; edgeIndex++)
		{
			unsigned int vertIndexA = edges(0, edgeIndex);
			unsigned int vertIndexB = edges(1, edgeIndex);

			edgeLengths[edgeIndex] = edgeLengthsOriginal[edgeIndex] * exp((deltaEdgeContrib[vertIndexA] + deltaEdgeContrib[vertIndexB]) / 2.f);
		}
	}

	// xxx: failure!!
}

void ShapeMorph::EmbedMesh(MatrixXf & points, const VectorXf & edgeLengths, const const VectorXi edges, const Matrix3Xi triangles)
{
	// 1. choose a point and direction

	// 2. while set of neighboring triangles is not empty:

		// embed neighboring triangle

}
