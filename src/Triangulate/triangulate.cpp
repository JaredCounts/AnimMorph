#include "triangulate.h"

#define REAL double
#define VOID int
#define ANSI_DECLARATORS

extern "C"
{
#include "triangle.h"
}

#undef ANSI_DECLARATORS
#undef VOID
#undef REAL

#include <iostream>

namespace Triangulate
{
	Mesh2D Triangulate(
		const Matrix2Xi &edges,
		const Matrix2Xf &vertices)
	{
		std::string full_flags = "a2.0qpz";

		typedef Map< Matrix<double, Dynamic, Dynamic, ColMajor> > MapXdr;
		typedef Map< Matrix<int, Dynamic, Dynamic, ColMajor> > MapXir;

		triangulateio in;
		in.numberofpoints = vertices.cols();
		in.pointlist = (double*)calloc(vertices.size(), sizeof(double));
		{
			MapXdr inpl(in.pointlist, vertices.rows(), vertices.cols());
			inpl = vertices.template cast<double>();
		}
		for (int i = 0; i < vertices.cols(); i++)
		{
			assert(in.pointlist[i * 2] == vertices(0, i));
			assert(in.pointlist[i * 2 + 1] == vertices(1, i));
		}

		in.numberofpointattributes = 0;
		in.pointmarkerlist = (int*)calloc(0, sizeof(int));

		in.trianglelist = NULL;
		in.numberoftriangles = 0;
		in.numberofcorners = 0;
		in.numberoftriangleattributes = 0;
		in.triangleattributelist = NULL;

		in.numberofsegments = edges.cols();
		in.segmentlist = (int*)calloc(edges.size(), sizeof(int));
		{
			MapXir insl(in.segmentlist, edges.rows(), edges.cols());
			insl = edges.template cast<int>();
		}

		for (int i = 0; i < edges.cols(); i++)
		{
			assert(in.segmentlist[i * 2] == edges(0, i));
			assert(in.segmentlist[i * 2 + 1] == edges(1, i));
		}

		in.segmentmarkerlist = (int*)calloc(edges.cols(), sizeof(int));
		for (int i = 0; i < edges.cols(); i++)
		{
			in.segmentmarkerlist[i] = 1;
		}

		in.numberofholes = 0;
		in.holelist = (double*)calloc(0, sizeof(double));
		
		in.numberofregions = 0;

		triangulateio out;

		out.pointlist = NULL;
		out.trianglelist = NULL;
		out.segmentlist = NULL;
		out.segmentmarkerlist = NULL;
		out.pointmarkerlist = NULL;

		::triangulate(const_cast<char*>(full_flags.c_str()), &in, &out, 0);

		Matrix2Xf outVertices = MapXdr(out.pointlist, 2, out.numberofpoints).cast<float>();
		Matrix3Xi outFaces = MapXir(out.trianglelist, 3, out.numberoftriangles);

		// Cleanup in
		free(in.pointlist);
		free(in.pointmarkerlist);
		free(in.segmentlist);
		free(in.segmentmarkerlist);
		free(in.holelist);

		// Cleanup out
		free(out.pointlist);
		free(out.trianglelist);
		free(out.segmentlist);
		free(out.segmentmarkerlist);
		free(out.pointmarkerlist);

		return Mesh2D(outVertices, outFaces);
	}
};