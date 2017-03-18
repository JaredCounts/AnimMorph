#include "renderer.h"

Renderer::Renderer(P_Camera camera) :
	camera(camera)
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

void Renderer::ReshapeDisplay(int width, int height)
{
	glViewport(0, 0, width, height);
}

void Renderer::Clear()
{
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);
}

#define GLSL(version, shader) "#version " #version "\n" #shader

void Renderer::Render(const Mesh2D & mesh)
{
	// XXX: a lot of this stuff doesn't need to be done every single frame
	//      at some point we may want to cache things
	const Matrix2Xf points = mesh.GetPoints();
	const Matrix3Xi triangles = mesh.GetTriangles();

	const char* vert = GLSL
	(
		410 core,
		layout(location = 0) in vec2 position;
		
		uniform mat4 modelViewProjection;

		void main()
		{
			gl_Position = modelViewProjection * vec4(position, 0.0, 1.0);
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

	// Get a handle for our "MVP" uniform
	GLuint matrixID = glGetUniformLocation(program, "modelViewProjection");

	Matrix4f projection = camera->projectionMatrix();
	Matrix4f view = camera->viewMatrix();
	Matrix4f model = Matrix4f::Identity();

	Matrix4f modelViewProjection = projection * view; // *model;

	// Send our transformation to the currently bound shader, 
	// in the "MVP" uniform
	glUniformMatrix4fv(matrixID, 1, GL_FALSE, modelViewProjection.data());


	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);


	glDeleteBuffers(1, &vertex_buffer);
	glDeleteBuffers(1, &index_buffer);
	glDeleteProgram(program);
	glDeleteVertexArrays(1, &VAO);

}

