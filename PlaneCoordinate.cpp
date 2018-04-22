#include "PlaneCoordinate.h"


PlaneCoordinate::PlaneCoordinate(int xRes, int yRes, double w, double h)
	:xRes(xRes), yRes(yRes), w(w), h(h) {
	initUvPoints();
};
void PlaneCoordinate::initUvPoints() {
	uvPoints = new double[2 * xRes * yRes];
	for (int i = 0; i < yRes; i++) {
		for (int j = 0; j < xRes; j++) {
			uvPoints[2 * (i*xRes + j)] = j * w / (xRes - 1);
			uvPoints[2 * (i*xRes + j) + 1] = i * h / (yRes - 1);
		}
	}
}
double *PlaneCoordinate:: getUvPoint(int i) { return uvPoints + 2 * i; }
double *PlaneCoordinate::getUvPoint(int x, int y) { return uvPoints + 2 * (y*xRes + x); }