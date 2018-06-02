#pragma once

#include <GL/freeglut.h>

#include "simulation.h"

#define FOV_Y 30.

#define M_PI 3.14f

class UI {
	Vector2d mouseLoc;

	int button;

public:
	int width, height;
	Simulation &sim;
	
	bool fillMode = true;
	bool colors = true;
	bool normals = true;

	UI(int, int, Simulation &);

	void render();
	void resize(int, int);


};
