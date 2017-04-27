#pragma once
#include "mouseable.h"

#include <vector>

class MouseManager
{
public:
	void MouseButtonChange(Mouseable::MouseButton buttonState);

	void MouseMove(int x, int y);

	void MouseScroll(double xOffset, double yOffset);

	void AddMouseable(P_Mouseable mouseable);

	int GetMouseX() const;
	int GetMouseY() const;

private:
	std::vector<P_Mouseable> mouseables;

	int mouseX, mouseY;
	Mouseable::MouseButton mouseButtonState;
};