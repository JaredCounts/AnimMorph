#include "renderer.h"

Renderer::Renderer()
{
}

bool Renderer::Init()
{
	glewExperimental = true;
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		std::cerr << "Glew init failed!" << std::endl;
		std::cerr << "Error: " << glewGetErrorString(err) << std::endl;
		return false;
	}
}

void Renderer::Clear()
{
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);
}

#define GLSL(version, shader) "#version " #version "\n" #shader

void Renderer::Render(const Mesh2D & mesh)
{

	const Matrix2Xf points = mesh.GetPoints();
	const Matrix3Xi triangles = mesh.GetTriangles();

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

	GLuint program = LoadProgram(vert, NULL, frag);

	GLuint VAO;
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	GLuint vertex_buffer = 0;
	glGenBuffers(1, &vertex_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);

	glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(float), points.data(), GL_STATIC_DRAW);

	GLuint index_buffer = 0;
	glGenBuffers(1, &index_buffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangles.size() * sizeof(int), triangles.data(), GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindVertexArray(0);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glUseProgram(program);
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);


	glDeleteBuffers(1, &vertex_buffer);
	glDeleteBuffers(1, &index_buffer);
	glDeleteProgram(program);
	glDeleteVertexArrays(1, &VAO);

}

