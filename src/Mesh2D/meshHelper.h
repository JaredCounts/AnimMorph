#pragma once

#include "mesh2D.h"

#include <map>

/*
MeshHelper

Keeps assists with topological look-up stuff.
*/
class MeshHelper
{
public:
	MeshHelper(const P_Mesh2D &mesh);

	/*
	vertIndexA: index of vertex A on edge
	vertIndexB: index of vertex B on edge

	returns: index of edge that contains vertex A and B.

	precondition: vertex A and B must be an existing edge.
	*/
	unsigned int EdgeToEdgeIndex(unsigned int vertIndexA, unsigned int vertIndexB) const;

	/*
	edgeIndex: index of edge

	returns: std::vector of (1 or 2) triangle indices that contain this edge.

	precondition: edge index must exist in mesh.
	*/
	const std::vector<unsigned int> &EdgeIndexToTriIndices(unsigned int edgeIndex) const;

	/*
	vertIndex: index of vertex

	returns: whether or not this vertex is on the boundary of the mesh

	precondition: vertex index must exist in the mesh.
	*/
	bool IsVertexOnBoundary(unsigned int vertIndex) const;

	/*
	triIndex: index of triangle

	returns: vector of triangle indices that border this triangle,
	not including this triangle. May be of length 0, 1, 2, or 3.

	precondition: triangle index must exist in this mesh
	*/
	const std::vector<unsigned int> &TriIndexToAdjTriIndices(unsigned int triIndex) const;

	/*
	triIndexA: index of triangle A
	triIndexB: index of triangle B

	returns: index of edge that these two triangles have in common

	precondition: both triangles need to share an edge

	*/
	unsigned int TriIndicesToCommonEdgeIndex(unsigned int triIndexA, unsigned int triIndexB) const;

	/*
	vertIndex: index of vertex
	edgeIndex: index of edge opposite of this vertex in triangle.

	returns: index of vertex opposite this edge.

	preconditions: vertex and edge must be of the same triangle.
				   edge must not include this vertex.
				   edge must contain 2 triangles.
	*/
	unsigned int OppVertIndexAcrossEdge(unsigned int vertIndex, unsigned int edgeIndex) const;

private:
	typedef std::pair<unsigned int, unsigned int> UIntPair;
	typedef std::vector<unsigned int> UIntVector;

	std::map<UIntPair, unsigned int> edgeToEdgeIndex;
	std::map<unsigned int, UIntVector> edgeIndexToTriIndices;
	std::map<unsigned int, bool> vertexOnBoundary;
	std::map<unsigned int, UIntVector> triIndexToAdjTriIndices;
	std::map<UIntPair, unsigned int> triIndicesToCommonEdgeIndex;

	// keyed by (vertIndex, edgeIndex) where edgeIndex is opposite to
	// vert index on the vert's triangle.
	std::map<UIntPair, unsigned int> oppVertIndexFromVertIndexAndEdgeIndex;
};