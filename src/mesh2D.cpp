#include "mesh2D.h"

#include <assert.h>

#include <iostream>

Mesh2D::Mesh2D()
{
}

void checkForErrors(std::string label)
{
	GLenum err;
	while ((err = glGetError()) != GL_NO_ERROR) {
		std::cerr << "(" << label << ") OpenGL error: " << err << std::endl;
	}
}

void Mesh2D::AddPoint(const Vector2f point)
{
	points.push_back(point);
}

void Mesh2D::AddPoints(const V_Vector2f points)
{
	for (auto &point : points)
	{
		AddPoint(point);
	}
}

void Mesh2D::AddTriangle(const Vector3i indices)
{
	// triangle indices must already be in mesh
	assert(indices[0] < points.size());
	assert(indices[1] < points.size());
	assert(indices[2] < points.size());

	triangles.push_back(indices);
}


void CheckStatus(GLuint obj)
{
	GLint status = GL_FALSE;
	if (glIsShader(obj)) glGetShaderiv(obj, GL_COMPILE_STATUS, &status);
	if (glIsProgram(obj)) glGetProgramiv(obj, GL_LINK_STATUS, &status);
	if (status == GL_TRUE) return;
	GLchar log[1 << 16] = { 0 };
	if (glIsShader(obj)) glGetShaderInfoLog(obj, sizeof(log), NULL, log);
	if (glIsProgram(obj)) glGetProgramInfoLog(obj, sizeof(log), NULL, log);
	std::cerr << log << std::endl;
	exit(-1);
}

void AttachShader(GLuint program, GLenum type, const char* src)
{
	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, &src, NULL);
	glCompileShader(shader);
	CheckStatus(shader);
	glAttachShader(program, shader);
	glDeleteShader(shader);
}

GLuint LoadProgram(const char* vert, const char* geom, const char* frag)
{
	GLuint prog = glCreateProgram();
	if (vert) AttachShader(prog, GL_VERTEX_SHADER, vert);
	if (geom) AttachShader(prog, GL_GEOMETRY_SHADER, geom);
	if (frag) AttachShader(prog, GL_FRAGMENT_SHADER, frag);
	glLinkProgram(prog);
	CheckStatus(prog);
	return prog;
}

#define GLSL(version, shader) "#version " #version "\n" #shader


void Mesh2D::Predraw()
{
	const char* vert = GLSL
	(
		410 core,
		layout(location = 0) in vec2 position;
	void main()
	{
		gl_Position = vec4(position, 0.0, 1.0);
	}
	);

	const char* frag = GLSL
	(
		410 core,
		out vec4 FragColor;
	void main()
	{
		FragColor = vec4(0.6, 1.0, 1.0, 1.0);
	}
	);

	program = LoadProgram(vert, NULL, frag);

	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	GLuint vertex_buffer = 0;
	glGenBuffers(1, &vertex_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
	float data[] =
	{
		0.0f,0.8f,
		-0.8f, 0.0f,
		0.8f,0.0f
	};

	const unsigned int verticesSize = points.size() * 2;
	GLfloat* vertices = new GLfloat[verticesSize];

	for (unsigned int i = 0; i < points.size(); i++)
	{
		// position
		vertices[i * 2] = points[i][0];
		vertices[i * 2 + 1] = points[i][1];
	
		//// color
		//vertices[i * 5 + 2] = 1.f;
		//vertices[i * 5 + 3] = 0.f;
		//vertices[i * 5 + 4] = 0.f;
	}

	glBufferData(GL_ARRAY_BUFFER, verticesSize * sizeof(GLfloat), vertices, GL_STATIC_DRAW);
	delete[] vertices;

	GLuint index_buffer = 0;
	glGenBuffers(1, &index_buffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);

	const unsigned int elementCount = 3 * triangles.size();
	GLuint *elements = new GLuint[elementCount];
	
	for (unsigned int i = 0; i < triangles.size(); i++)
	{
		elements[i * 3] = triangles[i][0];
		elements[i * 3 + 1] = triangles[i][1];
		elements[i * 3 + 2] = triangles[i][2];
	}

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, elementCount * sizeof(GLuint), elements, GL_STATIC_DRAW);
	delete[] elements;

	glEnableVertexAttribArray(0);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindVertexArray(0);
}

void Mesh2D::Draw()
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glUseProgram(program);
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}
