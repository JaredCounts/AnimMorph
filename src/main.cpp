#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <vector>
#include <iostream>
using namespace std;

#include "Mesh2D/mesh2D.h"
#include "Renderer/renderer.h"
#include "Camera/camera.h"

#include "Mouse/mouseManager.h"
#include "Mouse/mouseable.h"

#include "ShapeMorph\shapeMorph.h"

#include "Interpolation\linear.h"

MouseManager mouseManager;

P_Camera camera(new Camera());
P_Renderer renderer(new Renderer(camera));

int main(int argc, char** argv)
{
	// xxx: window logic and loop logic should be separate
	if (!glfwInit())
		return -1;

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	int width = 1080, height = 720;

	GLFWwindow* window = glfwCreateWindow(width, height, "Hello window", NULL, NULL);
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
	int trussWidth = 50;
	float trussHeight = 5;
	for (unsigned int j = 0; j < trussHeight; j++)
	{
		for (unsigned int i = 0; i < trussWidth; i++)
		{
			mesh.AddPoint(Vector2f(i, j));
		}
	}

	auto GetIndex = [trussWidth](int i, int j) {
		return i + j * trussWidth;
	};
	for (unsigned int i = 0; i < trussWidth - 1; i++)
	{
		for (unsigned int j = 0; j < trussHeight - 1; j++)
		{
			// make a square with (i,j), (i+1, j), (i+1,j+1), (i, j+1)
			int a = GetIndex(i, j);
			int b = GetIndex(i, j + 1);
			int c = GetIndex(i + 1, j + 1);
			int d = GetIndex(i + 1, j);
			mesh.AddTriangle(Vector3i(a, b, d));
			mesh.AddTriangle(Vector3i(b, c, d));
		}
	}
	

	// copy mesh
	Mesh2D mesh2(mesh);
	float angleStepSize = (2 * M_PI) / (trussWidth - 1);
	float radius = trussWidth / (2 * M_PI);
	for (int i = 0; i < trussWidth; i++)
	{
		for (int j = 0; j < trussHeight; j++)
		{
			int index = GetIndex(i, j);

			float angle = i * angleStepSize;
			float x = (radius + j) * cos(angle);
			float y = (radius + j) * sin(angle);
			mesh2.SetPoint(index, Vector2f(x, y));
		}
	}
	//for (unsigned int i = 0; i < trussWidth; i++)
	//{
	//	float angle = i * angleStepSize;
	//	float x = radius * cos(angle);
	//	float y = radius * sin(angle);
	//	mesh2.SetPoint(i*2, Vector2f(x, y));

	//	x = (radius + trussHeight) * cos(angle);
	//	y = (radius + trussHeight) * sin(angle);
	//	mesh2.SetPoint(i*2 + 1, Vector2f(x, y));
	//}

	Mesh2D mesh3(mesh);
	angleStepSize = 4 * (2 * M_PI) / (trussWidth - 1);
	radius = trussHeight;
	for (int i = 0; i < trussWidth; i++)
	{
		for (int j = 0; j < trussHeight; j++)
		{
			int index = GetIndex(i, j);

			float angle = i * angleStepSize;
			float x = i;
			float y = radius * sin(angle) + j;
			mesh3.SetPoint(index, Vector2f(x, y));
		}
	}
	//angleStepSize = 4 * (2 * M_PI) / (trussWidth - 1);
	//radius = trussHeight;
	//for (unsigned int i = 0; i < trussWidth; i++)
	//{
	//	float angle = i * angleStepSize;
	//	float x = i;
	//	float y = radius * sin(angle);
	//	mesh3.SetPoint(i * 2, Vector2f(x, y));

	//	x = i;
	//	y = radius * sin(angle) + trussHeight;
	//	mesh3.SetPoint(i * 2 + 1, Vector2f(x, y));
	//}

	float spacing = 50;
	mesh.Translate(Vector2f(0, 0));
	mesh2.Translate(Vector2f(0, 50));
	mesh3.Translate(Vector2f(0, 100));

	std::vector<Mesh2D> interpMeshes;
	std::cout << "compute mesh helper\n";
	MeshHelper meshHelper(mesh);
	for (int i = 0; i < 12; i++)
	{
		float t = i * (1.0 / 11);
		interpMeshes.push_back(ShapeMorph::Interpolate(mesh, mesh2, t, meshHelper, Interpolation::Linear));
		
		interpMeshes.back().Translate(spacing * Vector2f(i % 3, -(i / 3)));
	}

	camera->SetDimensions(width, height);
	camera->SetDistance(5);
	camera->SetCenter(Vector3f::Zero());

	renderer->Init();

	auto mouseButtonFunction = [](GLFWwindow *window, int button, int action, int mods)
	{
		Mouseable::MouseButton mouseButtonState;

		// determine mouse state
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

		// feed it into mouseManager
		mouseManager.MouseButtonChange(mouseButtonState);
	};
	
	auto mousePositionFunction = [](GLFWwindow *window, double x, double y)
	{
		mouseManager.MouseMove((int)x, (int)y);
	};

	auto mouseScrollFunction = [](GLFWwindow *window, double xOffset, double yOffset)
	{
		mouseManager.MouseScroll(xOffset, yOffset);
	};

	auto windowResizeFunction = [](GLFWwindow *window, int width, int height)
	{
		camera->SetDimensions(width, height);
		camera->SetViewport(0, 0, width, height);
		camera->SetPerspective(50);

		renderer->ReshapeDisplay(width, height);
	};

	windowResizeFunction(window, width, height);

	glfwSetMouseButtonCallback(window, mouseButtonFunction);
	glfwSetCursorPosCallback(window, mousePositionFunction);
	glfwSetScrollCallback(window, mouseScrollFunction);

	mouseManager.AddMouseable(camera);

	glfwSetWindowSizeCallback(window, windowResizeFunction);

	// Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0)
	{
		renderer->Clear();


		for (auto &mesh : interpMeshes)
		{
			renderer->Render(mesh);
		}
		
		renderer->Render(mesh);
		renderer->Render(mesh2);
		renderer->Render(mesh3);
		////t += 0.01;
		////mesh3 = ShapeMorph::Interpolate(mesh, mesh2, t);

		//renderer->Render(mesh3);

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
