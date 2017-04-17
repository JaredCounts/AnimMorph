#include "meshHelper.h"

MeshHelper::MeshHelper(const P_Mesh2D &mesh)
{
	const Matrix2Xi &edges = mesh->GetEdges();
	const unsigned int edgeCount = edges.cols();

	const Matrix3Xi &triangles = mesh->GetTriangles();
	const unsigned int triCount = triangles.cols();

	const unsigned int vertCount = mesh->PointCount();

	// std::unordered_map<unsigned int, unsigned int> edgeToEdgeIndex;
	// ====
	for (unsigned int edgeIndex = 0; edgeIndex < edgeCount; edgeIndex++)
	{
		unsigned int vertIndexA = edges(0, edgeIndex);
		unsigned int vertIndexB = edges(1, edgeIndex);

		UIntPair edge(vertIndexA, vertIndexB);
		UIntPair edgeReversed(vertIndexB, vertIndexA);

		edgeToEdgeIndex.emplace(edge, edgeIndex);
		edgeToEdgeIndex.emplace(edgeReversed, edgeIndex);
	}

	// std::unordered_map<unsigned int, std::vector<unsigned int>> edgeIndexToTriIndices;
	// ====
	for (unsigned int triIndex = 0; triIndex < triCount; triIndex++)
	{
		const Vector3i &triangle = triangles.col(triIndex);
		for (unsigned int innerEdgeIndex = 0; innerEdgeIndex < 3; innerEdgeIndex++)
		{
			UIntPair edge(triangle[innerEdgeIndex], triangle[(innerEdgeIndex + 1) % 3]);

			assert(edgeToEdgeIndex.find(edge) != edgeToEdgeIndex.end());

			const unsigned int edgeIndex = edgeToEdgeIndex.find(edge)->second;

			edgeIndexToTriIndices[edgeIndex].push_back(triIndex);
		}
	}

	// std::unordered_map<unsigned int, bool> vertexOnBoundary;
	// ====
	for (unsigned int vertIndex = 0; vertIndex < vertCount; vertIndex++)
	{
		vertexOnBoundary[vertIndex] = false;
	}
	for (unsigned int edgeIndex = 0; edgeIndex < edgeCount; edgeIndex++)
	{
		const unsigned int &vertIndexA = edges(0, edgeIndex);
		const unsigned int &vertIndexB = edges(1, edgeIndex);

		const unsigned int &adjTriangleCount = edgeIndexToTriIndices.find(edgeIndex)->second.size();

		// if there's only one triangle bordering this edge, then it's a boundary edge
		vertexOnBoundary[vertIndexA] =
			vertexOnBoundary[vertIndexA] || adjTriangleCount == 1;
		vertexOnBoundary[vertIndexB] =
			vertexOnBoundary[vertIndexB] || adjTriangleCount == 1;
	}

	// std::unordered_map<unsigned int, std::vector<unsigned int>> triIndexToAdjTriIndices;
	// std::unordered_map<UIntPair, unsigned int> triIndicesToCommonEdgeIndex
	// ====
	for (unsigned int triIndex = 0; triIndex < triCount; triIndex++)
	{
		const Vector3i &triangle = triangles.col(triIndex);
		for (unsigned int innerEdgeIndex = 0; innerEdgeIndex < 3; innerEdgeIndex++)
		{
			const unsigned int vertIndexA = triangle[innerEdgeIndex];
			const unsigned int vertIndexB = triangle[(innerEdgeIndex + 1) % 3];

			assert(edgeToEdgeIndex.find(UIntPair(vertIndexA, vertIndexB)) != edgeToEdgeIndex.end());

			const unsigned int edgeIndex = edgeToEdgeIndex.find(UIntPair(vertIndexA, vertIndexB))->second;

			const std::vector<unsigned int> &borderingTriangles = edgeIndexToTriIndices.find(edgeIndex)->second;

			if (borderingTriangles.size() == 1)
			{	// this triangle is the only one bordering this edge
				continue;
			}

			// can only be two triangles bordering an edge
			assert(borderingTriangles.size() == 2);

			unsigned int adjTriIndexA = borderingTriangles.at(0);
			unsigned int adjTriIndexB = borderingTriangles.at(1);

			unsigned int otherAdjTriIndex = adjTriIndexA;
			if (otherAdjTriIndex == triIndex)
			{
				otherAdjTriIndex = adjTriIndexB;
			}

			assert(otherAdjTriIndex != triIndex);

			triIndexToAdjTriIndices[triIndex].push_back(otherAdjTriIndex);

			triIndicesToCommonEdgeIndex[UIntPair(adjTriIndexA, adjTriIndexB)] = edgeIndex;
			triIndicesToCommonEdgeIndex[UIntPair(adjTriIndexB, adjTriIndexA)] = edgeIndex;
		}
	}

	// std::unordered_map<UIntPair, unsigned int> oppVertIndexFromVertIndexAndEdgeIndex;
	// ====
	for (unsigned int edgeIndex = 0; edgeIndex < edgeCount; edgeIndex++)
	{
		const unsigned int &vertIndexA = edges(0, edgeIndex);
		const unsigned int &vertIndexB = edges(1, edgeIndex);

		// vertIndexC will be the third vertex in any incident triangle

		const UIntVector &triIndices = edgeIndexToTriIndices[edgeIndex];

		if (triIndices.size() == 1)
		{	// there's no vertex opposite of vertIndexC
			continue;
		}

		// impossible for there to be 3 triangles incident to one edge
		assert(triIndices.size() == 2);

		const unsigned int &triIndexA = triIndices[0];
		const unsigned int &triIndexB = triIndices[1];

		unsigned int vertIndexC1 = vertIndexA;
		unsigned int innerVertIndex = 0;
		while (vertIndexC1 == vertIndexA || vertIndexC1 == vertIndexB)
		{
			assert(innerVertIndex < 3);

			vertIndexC1 = triangles(innerVertIndex, triIndexA);
			innerVertIndex += 1;
		}

		innerVertIndex = 0;
		unsigned int vertIndexC2 = vertIndexA;
		while (vertIndexC2 == vertIndexA || vertIndexC2 == vertIndexB)
		{
			assert(innerVertIndex < 3);

			vertIndexC2 = triangles(innerVertIndex, triIndexB);
			innerVertIndex += 1;
		}

		oppVertIndexFromVertIndexAndEdgeIndex[UIntPair(vertIndexC1, edgeIndex)] = vertIndexC2;
		oppVertIndexFromVertIndexAndEdgeIndex[UIntPair(vertIndexC2, edgeIndex)] = vertIndexC1;
	}
}

unsigned int MeshHelper::EdgeToEdgeIndex(unsigned int vertIndexA, unsigned int vertIndexB) const
{
	assert(edgeToEdgeIndex.find(UIntPair(vertIndexA, vertIndexB)) != edgeToEdgeIndex.end());
	return edgeToEdgeIndex.find(UIntPair(vertIndexA, vertIndexB))->second;
}

const std::vector<unsigned int> &MeshHelper::EdgeIndexToTriIndices(unsigned int edgeIndex) const
{
	assert(edgeIndexToTriIndices.find(edgeIndex) != edgeIndexToTriIndices.end());
	return edgeIndexToTriIndices.find(edgeIndex)->second;
}

unsigned int MeshHelper::TriIndicesToCommonEdgeIndex(unsigned int triIndexA, unsigned int triIndexB) const
{
	assert(triIndicesToCommonEdgeIndex.find(UIntPair(triIndexA, triIndexB)) != triIndicesToCommonEdgeIndex.end());
	return triIndicesToCommonEdgeIndex.find(UIntPair(triIndexA, triIndexB))->second;
}

bool MeshHelper::IsVertexOnBoundary(unsigned int vertIndex) const
{
	assert(vertexOnBoundary.find(vertIndex) != vertexOnBoundary.end());
	return vertexOnBoundary.find(vertIndex)->second;
}

const std::vector<unsigned int> &MeshHelper::TriIndexToAdjTriIndices(unsigned int triIndex) const
{
	assert(triIndexToAdjTriIndices.find(triIndex) != triIndexToAdjTriIndices.end());
	return triIndexToAdjTriIndices.find(triIndex)->second;
}

unsigned int MeshHelper::OppVertIndexAcrossEdge(unsigned int vertIndex, unsigned int edgeIndex) const
{
	assert(oppVertIndexFromVertIndexAndEdgeIndex.find(UIntPair(vertIndex, edgeIndex)) != oppVertIndexFromVertIndexAndEdgeIndex.end());
	return oppVertIndexFromVertIndexAndEdgeIndex.find(UIntPair(vertIndex, edgeIndex))->second;
}
