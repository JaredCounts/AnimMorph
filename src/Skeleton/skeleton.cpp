#include "skeleton.h"

#include <stack>
#include <algorithm>

Skeleton::Skeleton(const P_Mesh2D unposedMesh, P_Joint rootJoint) :
	unposedMesh(unposedMesh), rootJoint(rootJoint)
{
	// generate joints list
	std::stack<P_Joint> jointsToAdd;
	jointsToAdd.push(rootJoint);
	while (jointsToAdd.size() > 0)
	{
		P_Joint joint = jointsToAdd.top();
		jointsToAdd.pop();

		joints.push_back(joint);

		for (auto &childJoint : joint->children)
		{
			jointsToAdd.push(childJoint);
		}
	}

	// compute joint positions
	std::vector<Vector2f> jointPositions;
	for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
	{
		jointPositions.push_back(joints[jointIndex]->globalPoseTransform * Vector2f(0, 0));
	}

	// compute vertIndexToJointWeights
	const Matrix2Xf &points = unposedMesh->GetPoints_Local();
	for (int vertIndex = 0; vertIndex < unposedMesh->PointCount(); vertIndex++)
	{
		const Vector2f &vertex = points.col(vertIndex);

		float jointWeightSum = 0;
		for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
		{
			const Vector2f &joint = jointPositions[jointIndex];

			float dist = std::max((vertex - joint).norm(), 0.0001f);
			vertIndexToJointWeights[vertIndex][jointIndex] = log(1.0 / dist);
			jointWeightSum += vertIndexToJointWeights[vertIndex][jointIndex];
		}

		// sum over vertIndexToJointWeights[vertIndex][...] must = 1
		for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
		{
			vertIndexToJointWeights[vertIndex][jointIndex] /= jointWeightSum;
		}
	}
}

void ComputeJointTransforms(P_Joint joint, std::stack<Transform2f> &transformStack)
{
	joint->globalPoseTransform = transformStack.top() * joint->localPoseTransform;
	joint->poseToCurrentTransform = joint->globalPoseTransform.inverse() * joint->globalPoseTransform;

	transformStack.push(joint->globalPoseTransform);

	for (auto &child : joint->children)
	{
		ComputeJointTransforms(child, transformStack);
	}
}

P_Mesh2D Skeleton::PosedMesh()
{
	P_Mesh2D posedMesh(new Mesh2D(*unposedMesh));

	std::stack<Transform2f> transformStack;
	transformStack.push(Transform2f(Translation2f(0,0)));

	ComputeJointTransforms(rootJoint, transformStack);
	
	const Matrix2Xf &points = posedMesh->GetPoints_Local();
	for (unsigned int vertIndex = 0; vertIndex < posedMesh->PointCount(); vertIndex++)
	{
		Vector2f point = points.col(vertIndex);
		Vector2f transformedPoint(0, 0);

		for (unsigned int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
		{
			float weight = vertIndexToJointWeights[vertIndex][jointIndex];

			if (weight == 0)
			{
				continue;
			}

			transformedPoint += weight * (joints[jointIndex]->poseToCurrentTransform * point);
		}
		posedMesh->SetPoint(vertIndex, transformedPoint);
	}

	return posedMesh;
}