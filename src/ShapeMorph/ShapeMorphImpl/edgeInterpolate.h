#include "../../linearAlgebra.h"

#include "../../Mesh2D/mesh2D.h"

#include "../../Interpolation/interpolation.h"

namespace ShapeMorphImpl
{

	void InterpolateEdgeLengths(
		VectorXf &interpEdgeLengths,
		const P_Mesh2Ds &meshes,
		float t,
		const Interpolation::InterpolationFunc &interpolateFunction
	);

};