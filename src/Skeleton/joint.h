#pragma once
#include <memory>
#include <vector>

#include "../linearAlgebra.h"

struct Joint
{
	// initial pose transform relative to parent joint - this is the "from" transform
	Transform2f localPoseTransform;

	// initial pose transform relative to origin
	Transform2f globalPoseTransform;

	// final pose transform relative to parent joint - the "to" transform
	Transform2f localTransform;

	// to transform from -> to
	// we do globalPoseTransform.inverse() * globalTransform
	Transform2f poseToCurrentTransform;

	typedef std::shared_ptr<Joint> P_Joint;
	typedef std::vector<P_Joint> VP_Joints;

	VP_Joints children;
};

typedef std::shared_ptr<Joint> P_Joint;
typedef std::vector<P_Joint> VP_Joints;