#pragma once
#include "../Mesh2D/mesh2D.h"
#include "../Mesh2D/meshHelper.h"
#include "../Interpolation/interpolation.h"

namespace ShapeMorph
{

	P_Mesh2D Interpolate(
		const P_Mesh2Ds &meshes,
		float t,
		const MeshHelper &meshHelper,
		const Interpolation::InterpolationFunc &interpolateFunction);

};