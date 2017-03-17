#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <vector>
#include <iostream>
using namespace std;

#include "mesh2D.h"

int main(int argc, char** argv)
{
	if (!glfwInit())
		return -1;

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(640, 480, "Hello window", NULL, NULL);
	if (!window)
	{
		cerr << "Error on window creating" << endl;
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);

	cout << "GLFW version                : " << glfwGetVersionString() << endl;
	cout << "GLEW_VERSION                : " << glewGetString(GLEW_VERSION) << endl;
	cout << "GL_VERSION                  : " << glGetString(GL_VERSION) << endl;
	cout << "GL_VENDOR                   : " << glGetString(GL_VENDOR) << endl;
	cout << "GL_RENDERER                 : " << glGetString(GL_RENDERER) << endl;
	cout << "GL_SHADING_LANGUAGE_VERSION : " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;

	glewExperimental = true;
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		cerr << "Glew init failed!" << endl;
		cerr << "Error: " << glewGetErrorString(err) << endl;
	}


	Mesh2D mesh;
	mesh.AddPoint(Vector2f(0.0f, 0.8f));
	mesh.AddPoint(Vector2f(-0.8f, 0.0f));
	mesh.AddPoint(Vector2f(0.8f, 0.0f));
			
	mesh.AddTriangle(Vector3i(0, 1, 2));

	mesh.Predraw();

	// initData();


	// Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0)
	{
		glClearColor(0, 0, 0, 0);
		glClear(GL_COLOR_BUFFER_BIT);

		mesh.Draw();

		glfwSwapBuffers(window);

		glfwPollEvents();
		
		// check for errors
		GLenum err;
		while ((err = glGetError()) != GL_NO_ERROR) {
			std::cerr << "OpenGL error: " << err << std::endl;
		}

	}

	glfwTerminate();
	return 0;
}