#include "embedMesh.h"

#include <set>
#include <queue>
#include <iostream>

namespace ShapeMorphImpl
{

	void EmbedMesh(
		Matrix2Xf &points,
		const VectorXf &edgeLengths,
		const Matrix3Xi &triangles,
		const Matrix2Xi &edges,
		const MeshHelper &meshHelper,
		float firstTriangleAngle)
	{
		// std::cout << "embedding\n";
		// 1. choose a point and direction

		// 2. while set of neighboring triangles is not empty:

		// embed neighboring triangle

		// set all values in points to NaN
		points.setConstant(2, points.cols(), std::numeric_limits<float>::quiet_NaN());

		const int triangleCount = triangles.cols();

		// triangles that have been determined
		std::set<unsigned int> visitedTriIndices;
		// triangles that have been added to the triIndexQueue
		std::set<unsigned int> discoveredTriIndices;
		// triangles that are ready to be determined
		//     i.e. they already have two neighbors with determined vertices
		std::queue<unsigned int> triIndexQueue;

		// embed the first triangle;
		// std::cout << "\tEmbed first triangle.\n";
		{
			const Vector3i &triangle = triangles.col(0);

			visitedTriIndices.insert(0);
			discoveredTriIndices.emplace(0);

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

			// rotate it to match input angle
			points.col(vertIndexA) = Rotation2Df(firstTriangleAngle) * points.col(vertIndexA);
			points.col(vertIndexB) = Rotation2Df(firstTriangleAngle) * points.col(vertIndexB);
			points.col(vertIndexC) = Rotation2Df(firstTriangleAngle) * points.col(vertIndexC);

			// add all neighboring triangles to queue
			for (auto &triIndex : meshHelper.TriIndexToAdjTriIndices(0))
			{
				// the only tri index in the visited set is tri index 0
				// and that should not pop up in its adjacency vector
				assert(visitedTriIndices.find(triIndex) == visitedTriIndices.end());

				triIndexQueue.push(triIndex);
				discoveredTriIndices.emplace(triIndex);
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
				if (discoveredTriIndices.find(adjTriIndex) == discoveredTriIndices.end())
				{
					triIndexQueue.push(adjTriIndex);
					discoveredTriIndices.emplace(adjTriIndex);
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

}
