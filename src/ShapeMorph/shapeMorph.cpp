#include "shapeMorph.h"

#include <set>
#include <queue>
#include <iostream>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

Mesh2D ShapeMorph::Interpolate(
	const Mesh2D &start, 
	const Mesh2D &end, 
	float t, 
	const MeshHelper &meshHelper)
{
	// t must be between 0 and 1
	assert(0 <= t && t <= 1);

	// matrices must be topologically the same
	assert((start.GetTriangles() - end.GetTriangles()).norm() == 0);

	// 1. generate new edge lengths
	// std::cout << "1. generating new edge lengths.\n";
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

		assert(startEdgeLengthSq != 0);
		assert(endEdgeLengthSq != 0);

		interpEdgeLengths[i] = sqrt(abs((1 - t) * startEdgeLengthSq + t * endEdgeLengthSq));

		assert(interpEdgeLengths[i] != 0);
		assert(!std::isnan(interpEdgeLengths[i]));
		assert(std::isfinite(interpEdgeLengths[i]));
	}

	const VectorXf edgeLengthsCopy(interpEdgeLengths);

	const Matrix3Xi &triangles = start.GetTriangles();

	const unsigned int pointCount = start.PointCount();

	const unsigned int triangleCount = triangles.cols();

	// 2. flatten mesh using Conformal Equivalence of Triangle Mesh
	// std::cout << "2. flattening mesh.\n";
	FlattenEdgeLengths(interpEdgeLengths, edgeLengthsCopy, edges, triangles, pointCount, meshHelper);
	
	// std::cout << "Edge Lengths.\n============\n";
	// std::cout << interpEdgeLengths << '\n';

	// 3. Embed mesh into 2D Euclidean space
	// std::cout << "3. embedding mesh.\n";
	Matrix2Xf points = Matrix2Xf::Zero(2, pointCount);
	EmbedMesh(points, interpEdgeLengths, triangles, edges, meshHelper);

	return Mesh2D(points, triangles);
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

float Cot(const float &angle)
{
	return tan(M_PI / 2.0 - angle);
}

void ShapeMorph::FlattenEdgeLengths(
	VectorXf &edgeLengths, 
	const VectorXf &edgeLengthsOriginal,
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
	assert(edgeLengthsOriginal.size() == edgeCount);

	const unsigned int triangleCount = triangles.cols();

	// std::cout << "Generating coeff map.\n";
	// our system of equations only consists of vertices not on the boundary
	// (since the boundary vertices have their edgeContrib log values = 0
	// so we need a mapping from vert index to an index to and equation coeff
	unsigned int coeffCount = 0;
	// a coeff index of -1 means the vertex is a boundary vertex
	std::map<unsigned int, int> vertIndexToCoeffIndex;
	std::map<unsigned int, unsigned int> coeffIndexToVertIndex;

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
		std::cout << "\t1. Generate energy gradient.\n";
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
			// so our desiredAngleSum at each vertex is then 2PI, or PI for boundary vertices

			// 0.5 * (desiredAngleSum - angleSum)
			const unsigned int &vertIndex = coeffIndexToVertIndex[coeffIndex];

			energyGradient[coeffIndex] = 0.5 * (2 * M_PI - vertexAngles[vertIndex]);
			assert(!std::isnan(energyGradient[coeffIndex]) && std::isfinite(energyGradient[coeffIndex]));
		}
		
		assert(!energyGradient.hasNaN());

		if (energyGradient.squaredNorm() < Sq(0.00001))
		{	// stopping condition
			return;
		}

		// 2. Generate Energy Hessian
		std::cout << "\t2. Generate energy hessian.\n";
		SparseMatrix<float> energyHessian(coeffCount, coeffCount);

		// first we build a map of edge pairs to laplacian coefficients
		// then convert those to triplets
		// then load those into the sparse matrix (acc. to Eigen, this is the best way forward)

		std::map<UIntPair, float> halfLaplacianCoeffs;

		// the energy Hessian is the same as 1/2 the cot-Laplacian of the mesh
		for (unsigned int triIndex = 0; triIndex < triangleCount; triIndex++)
		{
			const Vector3i &triangle = triangles.col(triIndex);

			const unsigned int &vertIndexA = triangle[0];
			const unsigned int &vertIndexB = triangle[1];
			const unsigned int &vertIndexC = triangle[2];

			const unsigned int &coeffIndexA = vertIndexToCoeffIndex[vertIndexA];
			const unsigned int &coeffIndexB = vertIndexToCoeffIndex[vertIndexB];
			const unsigned int &coeffIndexC = vertIndexToCoeffIndex[vertIndexC];

			const float &triangleAngleA = triangleAngles(0, triIndex);
			const float &triangleAngleB = triangleAngles(1, triIndex);
			const float &triangleAngleC = triangleAngles(2, triIndex);

			const float quarterCotAngleA = 0.25 * Cot(triangleAngleA);
			const float quarterCotAngleB = 0.25 * Cot(triangleAngleB);
			const float quarterCotAngleC = 0.25 * Cot(triangleAngleC);


			if (coeffIndexA != -1 && coeffIndexB != -1)
			{
				halfLaplacianCoeffs[UIntPair(coeffIndexA, coeffIndexB)] += -quarterCotAngleC;
				halfLaplacianCoeffs[UIntPair(coeffIndexB, coeffIndexA)] += -quarterCotAngleC;
			}

			if (coeffIndexB != -1 && coeffIndexC != -1)
			{ 
				halfLaplacianCoeffs[UIntPair(coeffIndexB, coeffIndexC)] += -quarterCotAngleA;
				halfLaplacianCoeffs[UIntPair(coeffIndexC, coeffIndexB)] += -quarterCotAngleA;
			}

			if (coeffIndexA != -1 && coeffIndexC != -1)
			{
				halfLaplacianCoeffs[UIntPair(coeffIndexA, coeffIndexC)] += -quarterCotAngleB;
				halfLaplacianCoeffs[UIntPair(coeffIndexC, coeffIndexA)] += -quarterCotAngleB;
			}

			if (coeffIndexA != -1)
			{   // xxx: does this make sense?
				//      i.e. if vertex B isn't a coeff, 
				//           should it still contribute to the diagonal here?
				halfLaplacianCoeffs[UIntPair(coeffIndexA, coeffIndexA)] += quarterCotAngleB + quarterCotAngleC;
			}
			if (coeffIndexB != -1)
			{
				halfLaplacianCoeffs[UIntPair(coeffIndexB, coeffIndexB)] += quarterCotAngleA + quarterCotAngleC;
			}
			if (coeffIndexC != -1)
			{
				halfLaplacianCoeffs[UIntPair(coeffIndexC, coeffIndexC)] += quarterCotAngleA + quarterCotAngleB;
			}
		}

		typedef std::vector<Triplet<float>> V_Triplets;

		V_Triplets laplacianEntries;

		for (auto &lapEntry : halfLaplacianCoeffs)
		{
			const UIntPair &matrixPosition = lapEntry.first;
			const float &matrixCoeff = lapEntry.second;

			assert(!std::isnan(matrixCoeff) && std::isfinite(matrixCoeff));

			laplacianEntries.push_back(Triplet<float>(matrixPosition.first, matrixPosition.second, matrixCoeff));
		}

		energyHessian.setFromTriplets(laplacianEntries.begin(), laplacianEntries.end());
		
		// 3. Update "u" vector 
			// xxx: side remark, what do we call "u"?
			// they're the exponential edge contributions 
			// at each vector to the flattened edge
			// i.e. flatEdgeLength = e^(u[i] + u[j]) * edgeLength
			// so u = "vertexLogEdgeContribution"
		std::cout << "\t3. Update log edge contributions.\n";
		SparseQR<SparseMatrix<float>, COLAMDOrdering<int>> solver;

		// solver.analyzePattern(A);
		// solver.factorize(energyHessian);

		std::cout << "Energy Hessian\n==============\n" << energyHessian << "\n\n";
		std::cout << "Energy Gradient\n===============\n[" << energyGradient.norm() << "]\n" << energyGradient << "\n\n";
		solver.compute(energyHessian);
		assert(solver.info() == Success);

		VectorXf deltaEdgeContrib  = solver.solve(-0.25 * energyGradient);
		assert(solver.info() == Success);

		// center the deltaEdgeContrib values since the problem is scale invariant
		float average = deltaEdgeContrib.sum() / deltaEdgeContrib.size();
		deltaEdgeContrib = deltaEdgeContrib - average * VectorXf::Ones(pointCount);

		std::cout << "Delta Edge\n==========\n" << deltaEdgeContrib << "\n\n";

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

		if (deltaEdgeContrib.squaredNorm() < 0.0001)
		{
			return;
		}
	}

	// xxx: failure!!
	assert(false);
}

void ShapeMorph::EmbedMesh(
	Matrix2Xf &points, 
	const VectorXf &edgeLengths, 
	const Matrix3Xi &triangles,
	const Matrix2Xi &edges,
	const MeshHelper &meshHelper)
{
	// std::cout << "embedding\n";
	// 1. choose a point and direction

	// 2. while set of neighboring triangles is not empty:

		// embed neighboring triangle

	// set all values in points to NaN
	points.setConstant(2, points.cols(), std::numeric_limits<float>::quiet_NaN());

	const int triangleCount = triangles.cols();

	std::set<unsigned int> visitedTriIndices;
	std::queue<unsigned int> triIndexQueue;

	// embed the first triangle;
	// std::cout << "\tEmbed first triangle.\n";
	{
		const Vector3i &triangle = triangles.col(0);
		
		visitedTriIndices.insert(0);

		const unsigned int &vertIndexA = triangle[0];
		const unsigned int &vertIndexB = triangle[1];
		const unsigned int &vertIndexC = triangle[2];

		const unsigned int edgeIndexA = meshHelper.EdgeToEdgeIndex(vertIndexB, vertIndexC);
		const unsigned int edgeIndexB = meshHelper.EdgeToEdgeIndex(vertIndexA, vertIndexC);
		const unsigned int edgeIndexC = meshHelper.EdgeToEdgeIndex(vertIndexB, vertIndexA);

		const float &edgeLengthA = edgeLengths[edgeIndexA];
		const float &edgeLengthB = edgeLengths[edgeIndexB];
		const float &edgeLengthC = edgeLengths[edgeIndexC];

		points.col(vertIndexA) = Vector2f(0, 0);
		points.col(vertIndexB) = Vector2f(edgeLengthC, 0);
		float x = (Sq(edgeLengthB) + Sq(edgeLengthC) - Sq(edgeLengthA)) / (2 * edgeLengthC);
		float y = sqrt(Sq(edgeLengthB) - Sq(x));
		points.col(vertIndexC) = Vector2f(x, y);

		// add all neighboring triangles to queue
		for (auto &triIndex : meshHelper.TriIndexToAdjTriIndices(0))
		{
			// the only tri index in the visited set is tri index 0
			// and that should not pop up in its adjacency vector
			assert(visitedTriIndices.find(triIndex) == visitedTriIndices.end());

			triIndexQueue.push(triIndex);
		}
	}

	while (triIndexQueue.size() != 0)
	{
		// each triangle in the triIndexQueue should already have two of its points defined
		unsigned int triIndex = triIndexQueue.front();
		triIndexQueue.pop();
		visitedTriIndices.insert(triIndex);

		const Vector3i &triangle = triangles.col(triIndex);

		// std::cout << "\tAdd neighboring triangles.\n";
		// add all neighboring triangles to queue
		for (auto &adjTriIndex : meshHelper.TriIndexToAdjTriIndices(triIndex))
		{
			// the only tri index in the visited set is tri index 0
			// and that should not pop up in its adjacency vector
			if (visitedTriIndices.find(adjTriIndex) == visitedTriIndices.end())
			{
				triIndexQueue.push(adjTriIndex);
			}
		}

		// check to see that it has a third point that is NaN
		// (there's a case where if two of its neighbors were visited,
		//  then all three points will be already determined
		//  xxx: should we be checking that this implicitly determined 
		//       edge satisfies the given edge length?)

		// std::cout << "\tFind undetermined vertex.\n";
		unsigned int undetInnerVertIndex = 4;

		for (unsigned int innerVertIndex = 0; innerVertIndex < 3; innerVertIndex++)
		{
			if (std::isnan(points(0, triangle[innerVertIndex])))
			{
				// of course this means the y coordinate of this point should be undetermiend too.
				assert(std::isnan(points(1, triangle[innerVertIndex])));

				undetInnerVertIndex = innerVertIndex;

				break;
			}
		}

		if (undetInnerVertIndex == 4)
		{ // all three points are already determined
			continue;
		}

		const unsigned int undetVertIndex = triangle[undetInnerVertIndex];

		// the other two points should be determined
		assert(!std::isnan(points.col(triangle[(undetInnerVertIndex + 1) % 3])[0]));
		assert(!std::isnan(points.col(triangle[(undetInnerVertIndex + 1) % 3])[1]));
		assert(!std::isnan(points.col(triangle[(undetInnerVertIndex + 2) % 3])[0]));
		assert(!std::isnan(points.col(triangle[(undetInnerVertIndex + 2) % 3])[1]));

		/*
		   +-- adjVertexA
		   |
		   |     +--- undetVertex
		   v     v
		   x-----o							x = determined
		  / \   / <- current triangle		o = undetermined
		 /   \ /
		x-----x <-- adjVertexB
		^  ^----- bordering, already visited triangle: adjTriangle
		|
		+-- oppVertex

		we need all 3 x's shown here to determine the remaining 'o'
		*/

		int adjTriIndex = -1;

		unsigned int commonEdgeIndex, adjVertIndexA, adjVertIndexB;

		// determine already defined bordering triangle
		for (auto &otherTriIndex : meshHelper.TriIndexToAdjTriIndices(triIndex))
		{
			if (visitedTriIndices.find(otherTriIndex) != visitedTriIndices.end())
			{
				adjTriIndex = otherTriIndex;

				commonEdgeIndex 
					= meshHelper.TriIndicesToCommonEdgeIndex(triIndex, otherTriIndex);

				adjVertIndexA = edges(0, commonEdgeIndex);
				adjVertIndexB = edges(1, commonEdgeIndex);

				break;
			}
		}

		assert(adjTriIndex != -1);

		const Vector3i &adjTriangle = triangles.col(adjTriIndex);

		// std::cout << "\tDetermine opp. vertex.\n";
		unsigned int oppVertIndex = meshHelper.OppVertIndexAcrossEdge(undetVertIndex, commonEdgeIndex);

		// at this point we know the current triangle (triIndex/triangle), 
		// the undetermined vertex index in that triangle (undetInnerVertIndex),
		// the fully determined triangle bordering this triangle (adjTriIndex/adjTriangle)
		// the indices of the vertices that both triangles share (adjVertIndexA, adjVertIndexB)
		// the vertex on the bordering triangle opposite of the current triangle (oppVertIndex)

		const unsigned int &edgeIndexA = meshHelper.EdgeToEdgeIndex(undetVertIndex, adjVertIndexB);
		const unsigned int &edgeIndexB = meshHelper.EdgeToEdgeIndex(undetVertIndex, adjVertIndexA);
		const unsigned int &edgeIndexC = meshHelper.EdgeToEdgeIndex(adjVertIndexA, adjVertIndexB);

		const float &edgeLengthA = edgeLengths[edgeIndexA];
		const float &edgeLengthB = edgeLengths[edgeIndexB];
		const float &edgeLengthC = edgeLengths[edgeIndexC];

		// calculate its local 'x' and 'y' values
		// note, the basis is relative to adjVertA
		// so localY would be the distance from the point to edgeC
		// and localX would be the distance from adjVertA to point projected onto edgeC

		//std::cout << "\tCalculate x,y.\n";
		float localX = (Sq(edgeLengthB) + Sq(edgeLengthC) - Sq(edgeLengthA)) / (2 * edgeLengthC);
		float localY = sqrt(Sq(edgeLengthB) - Sq(localX));

		assert(!std::isnan(localX) && std::isfinite(localX));
		assert(!std::isnan(localY) && std::isfinite(localY));

		const Vector2f &adjVertA = points.col(adjVertIndexA);
		const Vector2f &adjVertB = points.col(adjVertIndexB);
		const Vector2f &oppVert = points.col(oppVertIndex);

		Vector2f edgeCDir = points.col(adjVertIndexB) - points.col(adjVertIndexA);
		edgeCDir.normalize();

		Vector2f edgeCPerpDir(-edgeCDir[1], edgeCDir[0]);

		Vector2f edgePoint = points.col(adjVertIndexA) + edgeCDir * localX;

		Vector2f vert = edgePoint + edgeCPerpDir * localY;

		// std::cout << "\tFix y orientation.\n";
		{	// make sure this triangle doesn't overlap the bordering triangle

			// xxx: there has gotta be a cleaner way to do this
			Vector3f adjVertAToVert = Vector3f::Zero();
			adjVertAToVert.head<2>() = vert - adjVertA;

			Vector3f adjVertBToVert = Vector3f::Zero();
			adjVertBToVert.head<2>() = vert - adjVertB;

			Vector3f adjVertAToOpp = Vector3f::Zero();
			adjVertAToOpp.head<2>() = oppVert - adjVertA;

			Vector3f adjVertBToOpp = Vector3f::Zero();
			adjVertBToOpp.head<2>() = oppVert - adjVertB;

			Vector3f triOrientation = adjVertAToVert.cross(adjVertBToVert);
			Vector3f oppTriOrientation = adjVertAToOpp.cross(adjVertBToOpp);

			if (triOrientation.dot(oppTriOrientation) > 0)
			{	// orientations are same sign -> therefore are vert is on the wrong side
				// std::cout << "\tflip\n";
				vert = edgePoint - edgeCPerpDir * localY;
			}
		}
		points.col(undetVertIndex) = vert;

		assert(!std::isnan(vert[0]) && !std::isnan(vert[1]));
		assert(std::isfinite(vert[0]) && std::isfinite(vert[1]));
	}

	// std::cout << "Points\n======\n";
	// std::cout << points << "\n\n";

	assert(visitedTriIndices.size() == triangleCount);

	assert(!points.hasNaN());
}
