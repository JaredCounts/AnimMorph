#include "mouseManager.h"

void MouseManager::MouseButtonChange(Mouseable::MouseButton buttonState)
{

	Mouseable::MouseButton prevMouseState = mouseButtonState;

	if (buttonState != prevMouseState)
	{
		if (buttonState == Mouseable::MouseButton::NONE)
		{ // mouse released
			for (auto &mouseable : mouseables)
			{
				mouseable->MouseRelease(mouseX, mouseY);
			}
		}
		else
		{ // mouse pressed
			for (auto &mouseable : mouseables)
			{
				mouseable->MouseClick(buttonState, mouseX, mouseY);
			}
		}
	}

	mouseButtonState = buttonState;
}

void MouseManager::MouseMove(int x, int y)
{
	int mousePrevX = mouseX;
	int mousePrevY = mouseY;

	mouseX = x;
	mouseY = y;

	if (mouseButtonState == Mouseable::MouseButton::NONE)
	{ // mouse motion
		for (auto &mouseable : mouseables)
		{
			mouseable->MouseMotion(mousePrevX, mousePrevY, mouseX, mouseY);
		}
	}
	else
	{ // mouse drag
		for (auto &mouseable : mouseables)
		{
			mouseable->MouseDrag(mouseButtonState, mousePrevX, mousePrevY, mouseX, mouseY);
		}
	}
}

void MouseManager::MouseScroll(double xOffset, double yOffset)
{
	for (auto &mouseable : mouseables)
	{
		mouseable->MouseScroll(xOffset, yOffset);
	}
}

void MouseManager::AddMouseable(P_Mouseable mouseable)
{
	mouseables.push_back(mouseable);
}
