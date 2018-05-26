#include "core.h"

int main(int argc, char **argv) {
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1000, 1000);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Cloth simulation");



	init();

	          glutDisplayFunc(display);
	//glutIdleFunc(idle);
	             glutReshapeFunc(resize);

	           glutMainLoop();
}
void init(GLvoid)
{
	glShadeModel(GL_SMOOTH);
	glClearColor(0.2f, 0.2f, 0.4f, 0.5f);
	glClearDepth(1.0f);
	//glClearColor(0, 0, 0, 0);
	glEnableClientState(GL_VERTEX_ARRAY);
	glShadeModel(GL_FLAT);
}


void initLights() {
	GLfloat lightPos0[3] = {1, 1, 0};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
	GLfloat lightAmb0[4] = {0.6f, 0.6f, 0.6f, 1};
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmb0);
	GLfloat lightDif0[4] = {0.6f, 0.6f, 0.6f, 1};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDif0);
}

void display() {
	sim.update();
	
	ui.render();
}


void resize(int w, int h) {
	ui.resize(w, h);
}
