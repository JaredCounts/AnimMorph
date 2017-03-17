#pragma once

#include "linearAlgebra.h"

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>


class Mesh2D
{
public:
	Mesh2D();

	void AddPoint(const Vector2f point);

	void AddPoints(const V_Vector2f points);

	void AddTriangle(const Vector3i indices);

	void Predraw();

	void Draw();

private:
	// geometry
	V_Vector2f points;

	// topology
	// each triangle contains 3 indices
	V_Vector3i triangles;

	GLuint program;
	GLuint VAO;
};