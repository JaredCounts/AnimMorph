#pragma once

#include <memory>

class Mouseable
{
public:
	typedef enum { NONE, LEFT, MIDDLE, RIGHT } MouseButton;

	virtual void MouseClick(MouseButton button, int x, int y) {};
	virtual void MouseRelease(int x, int y) {};
	virtual void MouseMotion(int prevX, int prevY, int x, int y) {};
	virtual void MouseDrag(MouseButton button, int prevX, int prevY, int x, int y) {};
};

typedef std::shared_ptr<Mouseable> P_Mouseable;