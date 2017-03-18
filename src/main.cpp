#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <vector>
#include <iostream>
using namespace std;

#include "mesh2D.h"
#include "renderer.h"
#include "Camera/camera.h"
#include "mouseable.h"



double mouseX, mouseY;
Mouseable::MouseButton mouseButtonState;
std::vector<P_Mouseable> mouseables;

void mouseButtonFunction(GLFWwindow *window, int button, int action, int mods)
{   // xxx should probably just pass this logic onto Mouseable
	Mouseable::MouseButton prevMouseState = mouseButtonState;

	if (action == GLFW_PRESS)
	{
		switch (button)
		{
		case GLFW_MOUSE_BUTTON_LEFT:
			if (mods & GLFW_MOD_CONTROL)
			{
				mouseButtonState = Mouseable::MouseButton::MIDDLE;
			}
			else if (mods & GLFW_MOD_ALT)
			{
				mouseButtonState = Mouseable::MouseButton::RIGHT;
			}
			else
			{
				mouseButtonState = Mouseable::MouseButton::LEFT;
			}
			break;
		case GLFW_MOUSE_BUTTON_MIDDLE:
			mouseButtonState = Mouseable::MouseButton::MIDDLE;
			break;
		case GLFW_MOUSE_BUTTON_RIGHT:
			mouseButtonState = Mouseable::MouseButton::RIGHT;
			break;
		default:
			mouseButtonState = Mouseable::MouseButton::NONE;
			break;
		}
	}
	else if (action == GLFW_RELEASE)
	{
		mouseButtonState = Mouseable::MouseButton::NONE;
	}

	if (mouseButtonState != prevMouseState)
	{
		if (mouseButtonState == Mouseable::MouseButton::NONE)
		{ // mouse released
			for (auto &mouseable : mouseables)
			{
				mouseable->MouseRelease((int)mouseX, (int)mouseY);
			}
		}
		else
		{ // mouse pressed
			for (auto &mouseable : mouseables)
			{
				mouseable->MouseClick(mouseButtonState, (int)mouseX, (int)mouseY);
			}
		}
	}

}
void mousePositionFunction(GLFWwindow *window, double x, double y)
{
	double mousePrevX = mouseX;
	double mousePrevY = mouseY;
	
	mouseX = x;
	mouseY = y;

	if (mouseButtonState == Mouseable::MouseButton::NONE)
	{ // mouse motion
		for (auto &mouseable : mouseables)
		{
			mouseable->MouseMotion((int)mousePrevX, (int)mousePrevY, (int)mouseX, (int)mouseY);
		}
	}
	else
	{ // mouse drag
		for (auto &mouseable : mouseables)
		{
			mouseable->MouseDrag(mouseButtonState, (int)mousePrevX, (int)mousePrevY, (int)mouseX, (int)mouseY);
		}
	}
}
void mouseEnterExitFunction(GLFWwindow *window, int entered)
{
	if (entered == GL_FALSE)
	{
		// act as a mouse button release
		mouseButtonFunction(window, -1, -1, -1);
	}
}

P_Camera camera(new Camera());

void windowResizeFunction(GLFWwindow *window, int width, int height)
{ 
	camera->SetDimensions(width, height);

	camera->SetViewport(0, 0, width, height);
	// camera->ApplyViewport();

	camera->SetPerspective(50);

	// xxx: this is renderer specific
	glViewport(0, 0, width, height);

	// renderer->ReshapeDisplay(width, height);
}

int main(int argc, char** argv)
{
	// xxx: window logic and loop logic should be separate
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


	Mesh2D mesh;
	mesh.AddPoint(Vector2f(0.0f, 0.8f));
	mesh.AddPoint(Vector2f(-0.8f, 0.0f));
	mesh.AddPoint(Vector2f(0.8f, 0.0f));
			
	mesh.AddTriangle(Vector3i(0, 1, 2));

	camera->SetDimensions(640, 480);
	camera->SetDistance(5);
	camera->SetCenter(Vector3f::Zero());

	mouseables.push_back(camera);

	windowResizeFunction(window, 640, 480);

	Renderer renderer(camera);
	renderer.Init();

	//int mousePrevX = 0;
	//int mousePrevY = 0;

	glfwSetMouseButtonCallback(window, mouseButtonFunction);
	glfwSetCursorPosCallback(window, mousePositionFunction);
	glfwSetCursorEnterCallback(window, mouseEnterExitFunction);

	glfwSetWindowSizeCallback(window, windowResizeFunction);

	// Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0)
	{
		renderer.Clear();
		renderer.Render(mesh);


		glfwSwapBuffers(window);

		glfwPollEvents();
		
		// check for errors
		// xxx: doesn't give much context when an error does happen
		GLenum err;
		while ((err = glGetError()) != GL_NO_ERROR) {
			std::cerr << "OpenGL error: " << err << std::endl;
		}

	}

	glfwTerminate();
	return 0;
}
