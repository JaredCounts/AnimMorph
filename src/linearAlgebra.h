#pragma once

#include <vector>

#include <Eigen/Dense>
using namespace Eigen;

#ifndef UP
#define UP  Vector3f(0,1,0)
#endif

// common typedefs

typedef std::vector<Vector2f> V_Vector2f;
typedef std::vector<Vector3f> V_Vector3f;

typedef std::vector<Vector2i> V_Vector2i;
typedef std::vector<Vector3i> V_Vector3i;