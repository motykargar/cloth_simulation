#include "ui.h"

UI::UI(int canvasWidth, int canvasHeight, Simulation &sim)
	: width(canvasWidth), height(canvasHeight), sim(sim) {
}



void UI::render() {
	GLfloat *tris = new GLfloat[9 * sim.getNumTris()];
	for (int i = 0; i < 9 * sim.getNumTris(); i++) {
		tris[i] = (GLfloat) sim.triVerts[i];
	}
	GLfloat *norms = new GLfloat[9 * sim.getNumTris()];
	for (int i = 0; i < 9 * sim.getNumTris(); i++) {
		norms[i] = (GLfloat) sim.norms[i];
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	gluPerspective(FOV_Y, 1. * width / height, 0.1, 100);

	
	glTranslatef(0, 0, -15);
	
	glRotatef(0, 0, 1, 0);
	glVertexPointer(3, GL_FLOAT, 0, tris);
	glNormalPointer(GL_FLOAT, 0, norms);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glColor3f(0, 1, 1);

	glDrawArrays(GL_TRIANGLES, 0, 3 * sim.getNumTris());

	glPopMatrix();
	glutSwapBuffers();
	
	glutPostRedisplay();
	delete tris;
	delete norms;
}

void UI::resize(int w, int h) {
	width  = w;
	height = h;
}

