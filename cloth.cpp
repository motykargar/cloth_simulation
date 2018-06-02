#include "cloth.h"

Cloth::Cloth(int xRes, int yRes, double mass, double w, double h)
	: xRes(xRes), yRes(yRes), mass(mass), w(w), h(h) {

	
	massPerVertI = (xRes * yRes) / mass;

	initUvPoints();
	initWorldPoints();
	initVelocities();
	
	initCuCv();
	initCuCvsave();
	inittriID();
	triUvArea = getTriUvArea();
	initmass();
}

// initialize uvPoints with uniform grid
void Cloth::initUvPoints() {
	uvPoints = new double[3 * xRes * yRes];
	
	for (int i = 0; i < yRes; i++) {
		for (int j = 0; j < xRes; j++) {
			uvPoints[3 * (i*xRes + j)]     = j * w / (xRes-1);
			uvPoints[3 * (i*xRes + j) + 1] = i * h / (yRes-1);
			uvPoints[3 * (i*xRes + j) + 2] = 0;
		}
	}
}

// initializte cloth world-state
void Cloth::initWorldPoints() {
	// init world positions
	worldPoints = new double[3 * xRes * yRes];
	worldPointssave = new double[3 * xRes * yRes];
	for (int i = 0; i < yRes; i++) {
		for (int j = 0; j < xRes; j++) {
			double *point = getUvPoint(j, i);
			setWorldPoint(j, i, point[0] - w / 2, point[1] - h / 2, 0);
			//setWorldPoint(j, i, point[0] , point[1] , 0);
			worldPoints[3 * (i*xRes + j)] = (j * w / (xRes - 1)) - w / 2;
			worldPoints[3 * (i*xRes + j) + 1] = (i * h / (yRes - 1)) - h / 2;
			worldPoints[3 * (i*xRes + j) + 2] = 0;
			

		}
	}
	for (int j = 0; j < 3 * xRes * yRes; j++) {
		worldPointssave[j] = worldPoints[j];
	}
}

void Cloth::initVelocities() {
	worldVels = new double[3 * xRes * yRes];
	lastDeltaV0 = new double[3 * xRes * yRes];
	worldVelssave = new double[3 * xRes * yRes];
	lastDeltaV0save = new double[3 * xRes * yRes];
	for (int i = 0; i < 3 * yRes*xRes; i++) {
		worldVels[i] = 0;
		lastDeltaV0[i] = 0;
		worldVelssave[i] = 0;
		lastDeltaV0save[i] = 0;
	}
}
void Cloth::initCuCv() {
	Cu = new double[2 * (xRes - 1) * (yRes - 1)];
	Cv = new double[2 * (xRes - 1) * (yRes - 1)];

	for (int i = 0; i < 2 * (xRes - 1) * (yRes - 1); i++) {
		Cu[i] = 0;
		Cv[i] = 0;
	

	}
}
void Cloth::initCuCvsave() {
	
	Cusave = new double[2 * (xRes - 1) * (yRes - 1)];
	Cvsave = new double[2 * (xRes - 1) * (yRes - 1)];
	for (int i = 0; i < 2 * (xRes - 1) * (yRes - 1); i++) {
	
		Cusave[i] = 0;
		Cvsave[i] = 0;

	}
}
void Cloth::initmass() {
	  Mass = new double[xRes * yRes];
	
	for (int i = 0; i < xRes * yRes; i++) {
		Mass[i] = DENSITY *triUvArea/3;
	}
}
void Cloth::inittriID() {
	triID.resize(2 * (xRes - 1) * (yRes - 1));
	for (int i = 0; i < 2 * (xRes - 1) * (yRes - 1); i++) {
		triID[i] = i;
	}
}
void Cloth::setWorldPoint(int xCor, int yCor, double xPos, double yPos,
	double zPos) {
	double *point = getWorldPoint(xCor, yCor);
	point[0] = xPos;
	point[1] = yPos;
	point[2] = zPos;
}

double Cloth::getTriUvArea() {
	return w * h / (2 * (xRes - 1) * (yRes - 1));
}

Vector3d Cloth::triNormal(int pi1, int pi2, int pi3) {
	auto p1 = Vector3d(getWorldPoint(pi1));

	// get side vectors
	auto s1 = Vector3d(getWorldPoint(pi2)) - p1;
	auto s2 = Vector3d(getWorldPoint(pi3)) - p1;

	return s1.cross(s2).normalized();
}
