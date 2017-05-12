#pragma once
#include <memory>
#include <vector>

#include "../linearAlgebra.h"

struct Joint
{
	// initial pose transform relative to parent joint - this is the "from" transform
	Transform2f unposedTransform_local;

	// initial pose transform relative to origin
	Transform2f unposedTransform_global;

	// final pose transform relative to parent joint - the "to" transform
	Transform2f posedTransform_local;

	// final pose transform relative to world
	Transform2f posedTransform_global;

	// to transform from -> to
	// we do globalUnposeTransform.inverse() * globalPosedTransform
	Transform2f unposedToCurrentTransform_global;

	typedef std::shared_ptr<Joint> P_Joint;
	typedef std::vector<P_Joint> VP_Joints;

	VP_Joints children;
};

typedef std::shared_ptr<Joint> P_Joint;
typedef std::vector<P_Joint> VP_Joints;