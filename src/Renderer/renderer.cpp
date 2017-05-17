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
	glClearColor(1, 1, 1, 0);
	glClear(GL_COLOR_BUFFER_BIT);
}

#define GLSL(version, shader) "#version " #version "\n" #shader

void Renderer::Render(const P_Mesh2D &mesh)
{
	// XXX: a lot of this stuff doesn't need to be done every single frame
	//      at some point we may want to cache things
	const Matrix2Xf points = mesh->GetPoints_World();
	const Matrix3Xi triangles = mesh->GetTriangles();

	Matrix3Xf colors(3, points.cols());
	for (int i = 0; i < points.cols(); i++)
	{
		colors.col(i) = mesh->color;
	}

	const char* vert = GLSL
	(
		330 core,

		layout(location = 0) in vec2 position;
		layout(location = 1) in vec3 vertexColor;

		out vec3 fragmentColor;

		uniform mat4 modelViewProjection;

		void main()
		{
			gl_Position = modelViewProjection * vec4(position, 0.0, 1.0);
			
			fragmentColor = vertexColor;
		}
	);

	const char* frag = GLSL
	(
		330 core,

		in vec3 fragmentColor;
	
		out vec3 color;
		
		void main()
		{
			color = fragmentColor;
		}
	);

	GLuint VAO;
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);
	//CheckStatus(VAO);

	GLuint program = LoadProgram(vert, NULL, frag);
	//CheckStatus(program);

	// vertices
	GLuint vertex_buffer;
	glGenBuffers(1, &vertex_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
	glBufferData(GL_ARRAY_BUFFER, 
		points.size() * sizeof(float), 
		points.data(), 
		GL_STATIC_DRAW);
	//CheckStatus(vertex_buffer);

	// colors
	GLuint color_buffer;
	glGenBuffers(1, &color_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, color_buffer);
	glBufferData(GL_ARRAY_BUFFER, 
		colors.size() * sizeof(float), 
		colors.data(), 
		GL_STATIC_DRAW);
	//CheckStatus(color_buffer);

	// triangles
	GLuint index_buffer = 0;
	glGenBuffers(1, &index_buffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 
		triangles.size() * sizeof(int), 
		triangles.data(), 
		GL_STATIC_DRAW);
	//CheckStatus(index_buffer);

	glUseProgram(program);

	// configure vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
	//glBindVertexArray(0);
	//CheckStatus(vertex_buffer);

	// configure colors
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, color_buffer);
	glVertexAttribPointer(
		1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);
	//glBindVertexArray(0);
	//CheckStatus(color_buffer);

	// set index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Get a handle for our "MVP" uniform
	GLuint matrixID = glGetUniformLocation(program, "modelViewProjection");

	Matrix4f projection = camera->projectionMatrix();
	Matrix4f view = camera->viewMatrix();
	Matrix4f model = Matrix4f::Identity();

	Matrix4f modelViewProjection = projection * view; // *model;

	// Send our transformation to the currently bound shader, 
	// in the "MVP" uniform
	glUniformMatrix4fv(matrixID, 1, GL_FALSE, modelViewProjection.data());

	//glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, triangles.size(), GL_UNSIGNED_INT, 0);
	//glBindVertexArray(0);
	
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	glDeleteBuffers(1, &index_buffer);
	glDeleteBuffers(1, &vertex_buffer);
	glDeleteBuffers(1, &color_buffer);

	glDeleteProgram(program);

	glDeleteVertexArrays(1, &VAO);
}

