#pragma once

#include "mesh2D.h"
#include "Camera/camera.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>

class Renderer
{
public:
	Renderer(P_Camera camera);

	bool Init();

	void Clear();

	void Render(const Mesh2D &mesh);

private:
	P_Camera camera;

	void CheckStatus(GLuint obj)
	{
		//GLint status = GL_FALSE;
		//if (glIsShader(obj)) glGetShaderiv(obj, GL_COMPILE_STATUS, &status);
		//if (glIsProgram(obj)) glGetProgramiv(obj, GL_LINK_STATUS, &status);
		//if (status == GL_TRUE) return;
		//GLchar log[1 << 16] = { 0 };
		//if (glIsShader(obj)) glGetShaderInfoLog(obj, sizeof(log), NULL, log);
		//if (glIsProgram(obj)) glGetProgramInfoLog(obj, sizeof(log), NULL, log);
		//std::cerr << log << std::endl;
		//exit(-1);
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

};