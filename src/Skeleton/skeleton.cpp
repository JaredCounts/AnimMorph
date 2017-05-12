#include "skeleton.h"

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

	std::stack<Transform2f> unposedTransformStack;
	unposedTransformStack.push(Transform2f(Translation2f(0, 0)));

	std::stack<Transform2f> poseTransformStack;
	poseTransformStack.push(Transform2f(Translation2f(0, 0)));

	ComputeJointTransforms(rootJoint, unposedTransformStack, poseTransformStack);

	for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
	{
		jointPositions.push_back(joints[jointIndex]->unposedTransform_global * Vector2f(0, 0));
	}

	// compute vertIndexToJointWeights
	vertIndexToJointWeights = std::vector<std::vector<float>>();
	const Matrix2Xf &points = unposedMesh->GetPoints_Local();
	for (int vertIndex = 0; vertIndex < unposedMesh->PointCount(); vertIndex++)
	{
		const Vector2f &vertex = points.col(vertIndex);

		vertIndexToJointWeights.push_back(std::vector<float>(joints.size(), 0));


		int closestJointIndex = 0, 
			closestJointIndex2 = 0;

		float closestDistance = std::numeric_limits<float>::infinity(), 
			closestDistance2 = std::numeric_limits<float>::infinity();

		for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
		{
			vertIndexToJointWeights[vertIndex][jointIndex] = 0;

			const Vector2f &jointPos = jointPositions[jointIndex];

			float distance = (vertex - jointPos).norm();
			if (distance < closestDistance)
			{
				closestDistance2 = closestDistance;
				closestJointIndex2 = closestJointIndex;

				closestDistance = distance;
				closestJointIndex = jointIndex;
			}
			else if (distance < closestDistance2)
			{
				closestDistance2 = distance;
				closestJointIndex2 = jointIndex;
			}
		}

		float distBtwnJoints = (jointPositions[closestJointIndex] - jointPositions[closestJointIndex2]).norm();

		float distDiff = closestDistance2 - closestDistance;
		float cutoff = distBtwnJoints * 0.3;
		//float cutoff = 0.3;
		//float distRatio = closestDistance / (closestDistance + closestDistance2);
		if (distDiff > cutoff)
		{
			vertIndexToJointWeights[vertIndex][closestJointIndex] = 1;
		}
		else
		{
			//float m = 1.0 / (cutoff - 1.0); //1 / (2 * cutoff - 1);dfdsafsdf
			//float b = -m; // 1 - m * cutoff;
			float m = 1 / (2 * cutoff); // 1.0 / (2.0 * (cutoff - distBtwnJoints)); ;//1.0 / (2.0 * cutoff - 1.0);
			float b = 1.0 - m * cutoff;

			vertIndexToJointWeights[vertIndex][closestJointIndex] = m * distDiff + b;
			vertIndexToJointWeights[vertIndex][closestJointIndex2] = 1 - vertIndexToJointWeights[vertIndex][closestJointIndex];

			assert(vertIndexToJointWeights[vertIndex][closestJointIndex] + vertIndexToJointWeights[vertIndex][closestJointIndex2] == 1);
		}

		//float jointWeightSum = 0;
		//for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
		//{
		//	const Vector2f &joint = jointPositions[jointIndex];

		//	float dist = std::max((vertex - joint).norm(), 0.0001f);
		//	vertIndexToJointWeights[vertIndex][jointIndex] = log(1.0 / dist);
		//	jointWeightSum += vertIndexToJointWeights[vertIndex][jointIndex];
		//}

		//// sum over vertIndexToJointWeights[vertIndex][...] must = 1
		//float maxWeight = -1;
		//int maxJointIndex;
		//for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
		//{
		//	vertIndexToJointWeights[vertIndex][jointIndex] /= jointWeightSum;
		//}

	}
}

void Skeleton::ComputeJointTransforms(
	P_Joint joint, 
	std::stack<Transform2f> &unposedTransformStack, 
	std::stack<Transform2f> &poseTransformStack)
{
	// XXX only need to compute unposed stuff once
	joint->unposedTransform_global = unposedTransformStack.top() * joint->unposedTransform_local;
	joint->posedTransform_global = poseTransformStack.top() * joint->posedTransform_local;

	joint->unposedToCurrentTransform_global = joint->posedTransform_global * joint->unposedTransform_global.inverse();

	unposedTransformStack.push(joint->unposedTransform_global);
	poseTransformStack.push(joint->posedTransform_global);

	for (auto &child : joint->children)
	{
		ComputeJointTransforms(child, unposedTransformStack, poseTransformStack);
	}

	unposedTransformStack.pop();
	poseTransformStack.pop();
}

P_Mesh2D Skeleton::PosedMesh()
{
	P_Mesh2D posedMesh(new Mesh2D(*unposedMesh));

	std::stack<Transform2f> unposedTransformStack;
	unposedTransformStack.push(Transform2f(Translation2f(0, 0)));

	std::stack<Transform2f> poseTransformStack;
	poseTransformStack.push(Transform2f(Translation2f(0, 0)));

	ComputeJointTransforms(rootJoint, unposedTransformStack, poseTransformStack);
	
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

			transformedPoint += weight * (joints[jointIndex]->unposedToCurrentTransform_global * point);
		}
		posedMesh->SetPoint(vertIndex, transformedPoint);
	}

	return posedMesh;
}