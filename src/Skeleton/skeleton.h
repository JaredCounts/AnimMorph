#pragma once

#include <unordered_map>

#include "joint.h"
#include "../Mesh2D/mesh2D.h"

class Skeleton
{
public:
	Skeleton(const P_Mesh2D unposedMesh, P_Joint rootJoint);

	P_Mesh2D PosedMesh();

private:
	P_Joint rootJoint;
	
	// keep a list of joints so we have their index to identify them by
	VP_Joints joints;

	// vertIndexToJointWeights[vertIndex][jointIndex] = weight of joint for vertex
	std::vector<std::vector<float>> vertIndexToJointWeights;
	
	const P_Mesh2D unposedMesh;
};