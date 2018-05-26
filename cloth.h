#pragma once

#include <Eigen/Dense>
#define DENSITY 0.1
using namespace Eigen;
struct Cloth {
	int xRes, yRes;
	double w, h;
	double *uvPoints;
	double *worldPoints;
	double *worldPointssave;
	double *Force;
	double *Forcesave;
	double *worldVels;
	double *lastDeltaV0;
	double *worldVelssave;
	double *lastDeltaV0save;
	double *Cu;
	double *Cv;
	double *Cusave;
	double *Cvsave;
	VectorXi triID;
	double massPerVertI; // inverted mass-per-vertex
	double *Mass;
	double mass;
	double triUvArea;
	Matrix3d *dforcex;
	Matrix3d *dforcev;

	
	Cloth(int, int, double, double, double);
	Cloth(int xRes, int yRes) : Cloth(xRes, yRes, 2.5,1, 1) {}

	void initUvPoints();
	void initWorldPoints();
	void initVelocities();
	void initForce();
	void initdForcex();
	void initdForcev();
	void initCuCv();
	void initCuCvsave();
	void inittriID();
	void initmass();
	
	double *getUvPoint(int i) { return uvPoints + 3 * i; }
	double *getUvPoint(int x, int y) { return uvPoints + 3 * (y*xRes + x);}
	double *getWorldPoint(int i) { return worldPoints + 3 * i; }
	double *getWorldPoint(int x, int y) { return worldPoints + 3 * (y*xRes + x);}
	double *getWorldVel(int i) { return worldVels + 3 * i; }
	double *getWorldVel(int x, int y){return worldVels + 3 * (y*xRes + x);}
	double *getlastDeltaV0(int i) { return lastDeltaV0 + 3 * i; }
	double *getlastDeltaV0(int x, int y) { return lastDeltaV0 + 3 * (y*xRes + x); }
	double *getForce(int i) { return Force + 3 * i; }
	double *getForce(int x, int y) { return Force + 3 * (y*xRes + x); }
	Matrix3d *getdForcex(int i) { return dforcex + i; }
	Matrix3d *getdForcex(int x, int y) { return dforcex +  (y*xRes + x); }
	Matrix3d *getdForcev(int i) { return dforcev +  i; }
	Matrix3d *getdForcev(int x, int y) { return dforcev +  (y*xRes + x); }


	void setWorldPoint(int, int, double, double, double);

	double getTriUvArea();
	Vector3d triNormal(int, int, int);
};
