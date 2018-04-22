#pragma once

//#include <GL/glew.h>
#include <GL/freeglut.h>

#include <stdio.h>

#include "ui.h"
#include "simulation.h"

Simulation sim(4, 4);

UI ui(720, 480, sim);

int main(int, char **);

void initLights();
void init(GLvoid);

void display();

void resize(int, int);
