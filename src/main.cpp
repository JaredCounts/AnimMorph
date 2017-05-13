#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <chrono>
#include <vector>
#include <iostream>
using namespace std;

#include "Mesh2D/mesh2D.h"
#include "Renderer/renderer.h"
#include "Camera/camera.h"

#include "Mouse/mouseManager.h"
#include "Mouse/mouseable.h"

#include "ShapeMorph\shapeMorph.h"
#include "ShapeMorph\ShapeMorphImpl\edgeInterpolate.h"

#include "Interpolation\linear.h"
#include "Interpolation\naturalCubicSpline.h"
#include "Interpolation\bezier.h"

#include "Mesh2D/loadMesh.h"

#include "Skeleton/skeleton.h"
#include "Skeleton/joint.h"


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

	P_Mesh2D mesh(new Mesh2D());
	int trussWidth = 50;
	float trussHeight = 5;
	for (unsigned int j = 0; j < trussHeight; j++)
	{
		for (unsigned int i = 0; i < trussWidth; i++)
		{
			mesh->AddPoint(Vector2f(i, j));
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
			mesh->AddTriangle(Vector3i(a, b, d));
			mesh->AddTriangle(Vector3i(b, c, d));
		}
	}
	

	// copy mesh
	P_Mesh2D mesh2(new Mesh2D(*mesh));
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
			mesh2->SetPoint(index, Vector2f(x, y));
		}
	}

	P_Mesh2D mesh3(new Mesh2D(*mesh));
	angleStepSize = 4 * (2 * M_PI) / (trussWidth - 1);
	radius = 0.5 * trussHeight;
	for (int i = 0; i < trussWidth; i++)
	{
		for (int j = 0; j < trussHeight; j++)
		{
			int index = GetIndex(i, j);

			float angle = i * angleStepSize;
			float x = i;
			float y = radius * sin(angle) + j;
			mesh3->SetPoint(index, Vector2f(x, y));
		}
	}

	float spacing = 15;
	mesh->Translate(Vector2f(0, 0));
	mesh2->Translate(Vector2f(0, 50));
	mesh3->Translate(Vector2f(0, 100));

	std::cout << "compute mesh helper\n";
	MeshHelper meshHelper(mesh);

	P_Mesh2Ds meshes;
	meshes.push_back(mesh);
	meshes.push_back(mesh2);
	//meshes.push_back(mesh3);
	//meshes.push_back(mesh);

	int meshCount = 600;

	typedef std::chrono::high_resolution_clock Time;
	typedef std::chrono::milliseconds ms;
	typedef std::chrono::duration<float> fsec;	

	std::cout << "start\n";
	auto t0 = Time::now();

	P_Mesh2Ds interpMeshes;
	//for (int i = 0; i < meshCount; i++)
	//{
	//	float t = (meshes.size()-1) * i * (1.0 / (meshCount - 1));
	//	interpMeshes.push_back(
	//		ShapeMorph::Interpolate(
	//			meshes, 
	//			t, 
	//			meshHelper, 
	//			Interpolation::CubicNaturalSplineFunc(true)));
	//}


	//P_Mesh2Ds interpMeshesLinear;
	//for (int i = 0; i < meshCount; i++)
	//{
	//	float t = (meshes.size() - 1) * i * (1.0 / (meshCount - 1));
	//	
	//	interpMeshesLinear.push_back(
	//		ShapeMorph::Interpolate(
	//			meshes, 
	//			t,
	//			meshHelper, 
	//			Interpolation::LinearFunc(true)));

	//	interpMeshesLinear.back()->Translate(Vector2f(spacing * i, 0));
	//}

	//P_Mesh2Ds interpMeshesBezier;

	//VectorXf start(meshHelper.GetEdges().cols());
	////VectorXf controlPointA(meshHelper.GetEdges().cols());
	////VectorXf controlPointB(meshHelper.GetEdges().cols());
	//VectorXf end(meshHelper.GetEdges().cols());

	//ShapeMorphImpl::InterpolateEdgeLengths(
	//	start,
	//	meshHelper.GetEdges(),
	//	meshes,
	//	0,
	//	Interpolation::CubicNaturalSplineFunc(false));

	//VectorXf controlDirA = start; // controlPointA - start;
	//VectorXf controlDirB = end; // controlPointA - end;

	//controlDirA = controlDirA - controlDirB * (controlDirA.dot(controlDirB)) / (controlDirA.norm() * controlDirB.norm());

	//controlDirA = controlDirA / controlDirA.norm();
	//controlDirB = controlDirB / controlDirB.norm();

	//float angleA = M_PI / 3;
	//float angleB = 2 * M_PI / 3;
	//VectorXf controlDirAF = controlDirA * cos(angleA) + controlDirB * sin(angleB);
	//VectorXf controlDirBF = controlDirA * cos(angleB) + controlDirB * sin(angleB);

	//Interpolation::InterpolationFunc bezierInterpFunc = Interpolation::BezierFunc(start + controlDirAF, end + controlDirBF);

	//for (int i = 0; i < meshCount; i++)
	//{
	//	float t = (meshes.size() - 1) * i * (1.0 / (meshCount - 1));
	//	std::cout << t << '\n';
	//	interpMeshesBezier.push_back(
	//		ShapeMorph::Interpolate(
	//			meshes,
	//			t,
	//			meshHelper,
	//			bezierInterpFunc));

	//	interpMeshesBezier.back()->Translate(Vector2f(spacing * i, 0));
	//}

	auto t1 = Time::now();
	fsec fs = t1 - t0;
	std::cout << "Finished. Took " << fs.count() << " seconds.\n";

	P_Mesh2D person(new Mesh2D(LoadMesh::LoadMesh("person.svg")));
	
	P_Joint root(new Joint);
	root->unposedTransform_local = Transform2f(Translation2f(0, -12));//0,-1.5));
	root->posedTransform_local = Transform2f(Translation2f(0, -12));

	P_Joint chest(new Joint);
	root->children.push_back(chest);
	chest->unposedTransform_local = Transform2f(Translation2f(0, 21)); // 0, 13.5));
	chest->posedTransform_local = Transform2f(Translation2f(0, 21));

	P_Joint neck(new Joint);
	chest->children.push_back(neck);
	neck->unposedTransform_local = Transform2f(Translation2f(0, 6)); // //0, 7));
	neck->posedTransform_local = Transform2f(Translation2f(0, 6));

	P_Joint head(new Joint);
	neck->children.push_back(head);
	head->unposedTransform_local = Transform2f(Translation2f(0, 8));
	head->posedTransform_local = Transform2f(Translation2f(0, 8));
	
	P_Joint shoulderLeft(new Joint);
	chest->children.push_back(shoulderLeft);
	shoulderLeft->unposedTransform_local = Transform2f(Translation2f(-10, -0.5));// -5, -3.25));
	shoulderLeft->posedTransform_local = Transform2f(Translation2f(-10, -0.5));

	P_Joint elbowLeft(new Joint);
	shoulderLeft->children.push_back(elbowLeft);
	elbowLeft->unposedTransform_local = Transform2f(Translation2f(-5, -7)); // -7.5, -3.25));
	elbowLeft->posedTransform_local = Transform2f(Translation2f(-5, -7));

	P_Joint handLeft(new Joint);
	elbowLeft->children.push_back(handLeft);
	handLeft->unposedTransform_local = Transform2f(Translation2f(-5, -7.5)); // -5, -7.75));
	handLeft->posedTransform_local = Transform2f(Translation2f(-5, -7.5));
	float angle = 1;


	P_Joint shoulderRight(new Joint);
	chest->children.push_back(shoulderRight);
	shoulderRight->unposedTransform_local = Transform2f(Translation2f(10, -0.5)); // 5, -3.25));
	shoulderRight->posedTransform_local = Transform2f(Translation2f(10, -0.5));

	P_Joint elbowRight(new Joint);
	shoulderRight->children.push_back(elbowRight);
	elbowRight->unposedTransform_local = Transform2f(Translation2f(5, -7)); // 7.5, -3.25));
	elbowRight->posedTransform_local = Transform2f(Translation2f(5, -7));

	P_Joint handRight(new Joint);
	elbowRight->children.push_back(handRight);
	handRight->unposedTransform_local = Transform2f(Translation2f(5, -7.5)); // 5, -7.75));
	handRight->posedTransform_local = Transform2f(Translation2f(5, -7.5));

	P_Joint hipLeft(new Joint);
	root->children.push_back(hipLeft);
	hipLeft->unposedTransform_local = Transform2f(Translation2f(-4,-4)); // -2, -12.5));
	hipLeft->posedTransform_local = Transform2f(Translation2f(-4, -4));

	P_Joint kneeLeft(new Joint);
	hipLeft->children.push_back(kneeLeft);
	kneeLeft->unposedTransform_local = Transform2f(Translation2f(-1,-11)); // -2.5, -7.5));
	kneeLeft->posedTransform_local = Transform2f(Translation2f(-1, -11));

	P_Joint footLeft(new Joint);
	kneeLeft->children.push_back(footLeft);
	footLeft->unposedTransform_local = Transform2f(Translation2f(0, -11)); // -0.5, -11));
	footLeft->posedTransform_local = Transform2f(Translation2f(0, -11));

	P_Joint hipRight(new Joint);
	root->children.push_back(hipRight);
	hipRight->unposedTransform_local = Transform2f(Translation2f(4, -4)); // 2, -12.5));
	hipRight->posedTransform_local = Transform2f(Translation2f(4, -4));

	P_Joint kneeRight(new Joint);
	hipRight->children.push_back(kneeRight);
	kneeRight->unposedTransform_local = Transform2f(Translation2f(1, -11)); // 2.5, -7.5));
	kneeRight->posedTransform_local = Transform2f(Translation2f(1, -11));

	P_Joint footRight(new Joint);
	kneeRight->children.push_back(footRight);
	footRight->unposedTransform_local = Transform2f(Translation2f(0, -11)); // 0.5, -11));
	footRight->posedTransform_local = Transform2f(Translation2f(0, -11));

	chest->posedTransform_local.prerotate(-M_PI / 48);

	handRight->posedTransform_local.prerotate(M_PI / 8);
	handLeft->posedTransform_local.prerotate(M_PI / 8);
	
	neck->posedTransform_local.prerotate(M_PI / 16);

	kneeLeft->posedTransform_local.prerotate(-M_PI / 8);
	footLeft->posedTransform_local.prerotate(M_PI / 8);

	kneeRight->posedTransform_local.prerotate(M_PI / 8);
	footRight->posedTransform_local.prerotate(-M_PI / 8);
	Skeleton skeleton(person, root);

	P_Mesh2D posedPerson = skeleton.PosedMesh();
	P_Mesh2Ds persons;
	persons.push_back(person);
	persons.push_back(posedPerson);
	//persons.push_back(person);

	MeshHelper personMeshHelper(person);
	for (int i = 0; i < meshCount; i++)
	{
		float t = (persons.size() - 1.0) * i * (1.0 / (meshCount - 1));
		std::cout << t << '\n';

		interpMeshes.push_back(
			ShapeMorph::Interpolate(
				persons,
				t,
				personMeshHelper,
				Interpolation::CubicNaturalSplineFunc(true)));
	}

	P_Mesh2Ds interpMeshesLinear;
	for (int i = 0; i < meshCount; i++)
	{
		float t = (persons.size() - 1.0) * i * (1.0 / (meshCount - 1));
		
		interpMeshesLinear.push_back(
			ShapeMorph::Interpolate(
				persons,
				t,
				personMeshHelper,
				Interpolation::LinearFunc(true)));

		interpMeshesLinear.back()->Translate(Vector2f(spacing * 3, 0));
	}

	P_Mesh2Ds interpMeshesBezier;

	VectorXf start(personMeshHelper.GetEdges().cols());
	VectorXf end(personMeshHelper.GetEdges().cols());
	//VectorXf controlPointA(meshHelper.GetEdges().cols());
	//VectorXf controlPointB(meshHelper.GetEdges().cols());

	// cheat to get edge lengths at 0
	ShapeMorphImpl::InterpolateEdgeLengths(
		start,
		personMeshHelper.GetEdges(),
		persons,
		0.0,
		Interpolation::LinearFunc(false));
	// cheat to get edge lengths at 1
	ShapeMorphImpl::InterpolateEdgeLengths(
		end,
		personMeshHelper.GetEdges(),
		persons,
		1.0,
		Interpolation::LinearFunc(false));

	VectorXf controlDirA = start.normalized(); // controlPointA - start;
	VectorXf controlDirB = end.normalized(); // controlPointA - end;

	//controlDirA.normalize(); // = controlDirA / controlDirA.norm();
	//controlDirB.normalize(); // = controlDirB / controlDirB.norm();

	// gram-schmidt
	controlDirA -= controlDirB * controlDirA.dot(controlDirB);
	controlDirA.normalize();

	float angleA = 2 * M_PI / 3;
	float angleB = M_PI / 3;
	VectorXf controlDirAF = controlDirA * cos(angleA) + controlDirB * sin(angleA);
	VectorXf controlDirBF = controlDirA * cos(angleB) + controlDirB * sin(angleB);

	Interpolation::InterpolationFunc bezierInterpFunc 
		= Interpolation::BezierFunc(3 * start + 2 * controlDirAF, 3 * end + 2 * controlDirBF);

	for (int i = 0; i < meshCount; i++)
	{
		float t = (persons.size() - 1) * i * (1.0 / (meshCount - 1));
		std::cout << t << "\n";
		interpMeshesBezier.push_back(
			ShapeMorph::Interpolate(
				persons,
				t,
				personMeshHelper,
				bezierInterpFunc));

		interpMeshesBezier.back()->Translate(Vector2f(spacing * 6, 0));
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
			std::cout << camera->GetPointWorld(Vector2f(mouseManager.GetMouseX(), mouseManager.GetMouseY())) << "\n\n";
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

	int meshIndex = 0;
	// Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
		glfwWindowShouldClose(window) == 0)
	{
		angle += 0.01;
		// head->posedTransform_local.translation() = Rotation2Df(0.01) * head->posedTransform_local.translation(); //.rotate(0.01);
		// head->posedTransform_local.prerotate(0.01);
		// head->posedTransform_local = Transform2f(Translation2f(0,12) * Rotation2Df(angle));
		//handLeft->posedTransform_local.prerotate(-0.01);
		//handLeft->posedTransform_local = Transform2f(Rotation2Df(angle) * Translation2f(-5, -7.5)); // * ;
		
		renderer->Clear();

		renderer->Render(interpMeshes[meshIndex]);
		renderer->Render(interpMeshesLinear[meshIndex]);
		renderer->Render(interpMeshesBezier[meshIndex]);
		meshIndex = (meshIndex + 1) % interpMeshes.size();


		//renderer->Render(person);
		
		//renderer->Render(skeleton.PosedMesh());

		//for (auto &mesh : interpMeshes)
		//{
		//	renderer->Render(mesh);
		//}
		//
		//for (auto &mesh : interpMeshesBezier)
		//{
		//	renderer->Render(mesh);
		//}

		//float mouseX = mouseManager.GetMouseX();
		//float mouseY = mouseManager.GetMouseY();
		//float centerX = camera->GetWidth() / 2;
		//float centerY = camera->GetHeight() / 2;
		//std::cout << mouseX << ", " << mouseY << "; " << centerX << ", " << centerY << '\n';
		//float mag = (Vector2f(mouseX, mouseY) - Vector2f(centerX, centerY)).norm() * 0.06;
		////angle += 0.05;
		//angle = -atan2(mouseY - centerY, mouseX - centerX);
		//float angleA = angle; // +M_PI / 2;
		//float angleB = 0; // -0.7 * angle + 2 * M_PI / 2;
		//VectorXf controlDirAF = controlDirA * cos(angleA) + controlDirB * sin(angleB);
		//VectorXf controlDirBF = controlDirA * cos(angleB) + controlDirB * sin(angleB);

		//Interpolation::InterpolationFunc bezierInterpFunc = Interpolation::BezierFunc(start + mag * controlDirAF, end + controlDirBF); // mag * 

		//for (int i = 0; i < meshCount; i++)
		//{
		//	float t = (meshes.size() - 1) * i * (1.0 / (meshCount - 1));
		//	interpMeshesBezier[i] = (
		//		ShapeMorph::Interpolate(
		//			meshes,
		//			t,
		//			meshHelper,
		//			bezierInterpFunc));

		//	interpMeshesBezier[i]->Translate(Vector2f(spacing * i, 0));
		//}

		//renderer->Render(mesh);
		//renderer->Render(mesh2);
		//renderer->Render(mesh3);
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
