#include "skeleton.h"

#include "../Geodesic/geodesic.h"

#include <algorithm>

#include <queue>

Skeleton::Skeleton(const P_Mesh2D unposedMesh, P_Joint rootJoint) :
	unposedMesh(unposedMesh), rootJoint(rootJoint)
{
	// generate joints list
	// sorted topologically (root -> children -> children's children -> ...)
	std::queue<P_Joint> jointsToAdd;
	jointsToAdd.push(rootJoint);
	while (jointsToAdd.size() > 0)
	{
		P_Joint joint = jointsToAdd.front();
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

	const Matrix2Xf &points = unposedMesh->GetPoints_Local();

	// row -> vertex
	// column -> joint
	MatrixXf distancesToJoints(unposedMesh->PointCount(), joints.size());
	for (int jointIndex = 0; jointIndex < joints.size(); jointIndex++)
	{
		const Vector2f &jointPos = jointPositions[jointIndex];

		// need to find nearest vertex to this joint
		int closestVertIndex = 0;
		float closestVertDistSq = std::numeric_limits<float>::infinity();
		for (int vertIndex = 0; vertIndex < unposedMesh->PointCount(); vertIndex++)
		{
			const Vector2f &vertex = points.col(vertIndex);

			float distSq = (vertex - jointPos).squaredNorm();

			if (distSq < closestVertDistSq)
			{
				closestVertIndex = vertIndex;
				closestVertDistSq = distSq;
			}
		}

		distancesToJoints.col(jointIndex) = Geodesic::DistanceFrom(closestVertIndex, points, unposedMesh->GetTriangles());
	}

	
	// compute vertIndexToJointWeights
	vertIndexToJointWeights = std::vector<std::vector<float>>();
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

			//const Vector2f &jointPos = jointPositions[jointIndex];

			float distance = distancesToJoints(vertIndex, jointIndex); // (vertex - jointPos).norm();
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

		int lowerJointIndex = std::max(closestJointIndex, closestJointIndex2);
		int upperJointIndex = std::min(closestJointIndex, closestJointIndex2);

		float lowerJointDist = lowerJointIndex == closestJointIndex ? closestDistance : closestDistance2;
		float upperJointDist = lowerJointIndex == closestJointIndex ? closestDistance2 : closestDistance;


		float distDiff = upperJointDist - lowerJointDist;
		// distDiff goes from [-distBtwnJoints,distBtwnJoints]
		// to make it easier on us, we'll scale it so that it goes from [0,1]

		float cutoffLower = 0.9; // fraction from upper to lower to "cuttoff" 100% transformation
		float cutoffUpper = 0;
		float weightParam = (distDiff + distBtwnJoints) / (2 * distBtwnJoints);
		
		if (weightParam > cutoffLower)
		{
			vertIndexToJointWeights[vertIndex][lowerJointIndex] = 1;
		}
		else
		{
			float m = 1.0 / (cutoffLower - cutoffUpper);
			float b = -m * cutoffUpper;

			vertIndexToJointWeights[vertIndex][lowerJointIndex] = m * weightParam + b;
			vertIndexToJointWeights[vertIndex][upperJointIndex] = 1.0 - vertIndexToJointWeights[vertIndex][lowerJointIndex];
			
			//assert(vertIndexToJointWeights[vertIndex][lowerJointIndex] + vertIndexToJointWeights[vertIndex][upperJointIndex] == 1.0);
		}

		


		//float distDiff = closestDistance2 - closestDistance;
		//float cutoff = distBtwnJoints * 0.9;
		//if (distDiff > cutoff)
		//{
		//	vertIndexToJointWeights[vertIndex][closestJointIndex] = 1;
		//}
		//else
		//{
		//	float m = 1 / (2 * cutoff);
		//	float b = 1.0 - m * cutoff;

		//	vertIndexToJointWeights[vertIndex][closestJointIndex] = m * distDiff + b;
		//	vertIndexToJointWeights[vertIndex][closestJointIndex2] = 1 - vertIndexToJointWeights[vertIndex][closestJointIndex];

		//	assert(vertIndexToJointWeights[vertIndex][closestJointIndex] + vertIndexToJointWeights[vertIndex][closestJointIndex2] == 1);
		//}

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