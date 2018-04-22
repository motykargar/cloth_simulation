#pragma once

struct PlaneCoordinate
{
	int xRes, yRes;
	double w, h;
	double *uvPoints;
	
	void initUvPoints();
	double *getUvPoint(int ) {}
	double *getUvPoint(int , int ) {}

	PlaneCoordinate(int, int, double, double);
};
