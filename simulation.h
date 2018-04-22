#pragma once

#include <Eigen/Sparse>

#include <string.h>
#include <iostream>
#include <random>

#include "cloth.h"
#include "Force.h"
#include "ForceBend.h"
#include "dForcex.h"
#include "dForcev.h"
#include "dFbendx.h"
#include "dFbendv.h"
#include "Eforce.h"
#include "EFbend.h"
#include "Ctri.h"

using namespace Eigen;
#define LOCK_TOP_ROW true

#define GRAVITY_ENABLED true
#define FLOOR_ENABLED false
#define ACCEL_LOCK true
#define STRETCHLIMIT  0.03
#define DENSITY 0.01
#define STRETCH_STIFF 5000
#define DAMP_STIFF  100
#define SHEAR_STIFF 500
#define BEND_STIFF 0.00001
#define IT 5

#define TIMESTEP 0.02



#define FLOOR_HEIGHT -1

#define MAX_SCALE 0.000000001
#define MAX_BEND 0.000003

typedef Matrix<double, Dynamic, Dynamic> ForceMatrix;
typedef Matrix<double, 3, 3> dForceMatrix;
typedef Matrix<double, Dynamic, Dynamic> DForceMatrix;
class Simulation {
	int h;
	int trii;
	int triii;
	double maxScale;
	double scaleX;
	double scaleY;
	

	void fShearStretch(int);
	void fShearStretchHelper(int *);
	void dfShearStretchHelper(int *);
	void EfShearStretchHelper(int *);
	void CuCv(int *);
	

	

	void fBend(int);
	void bendHelper(int *);
	void dfbendHelper(int *);
	void EfbendHelper(int *);

	double *genTrisFromMesh();
	double *genNorms();
	double *genTriNorms();
	void copyPoint(double *, double *);

public:
	Cloth cloth;
	double *triVerts;
	double *norms;
	bool running = true;
	bool bannerStyle = false;
	bool cuffing = false;
	
	//ForceMatrix forces0;
	double mas;
	DForceMatrix a;
	DForceMatrix M;
	ForceMatrix V;
	ForceMatrix b;
	ForceMatrix LM;
	DForceMatrix S;
	DForceMatrix z0;
	ForceMatrix deltaV;
	ForceMatrix forces;
	DForceMatrix dforcesv;
	DForceMatrix dforcesx;
	Matrix<double, 5, 1> fenergy;  // stretch, shear, bend, gravity, drag.
	Matrix<double, 5, Dynamic> trienergy;
	Matrix<double, 1, Dynamic> CU;
	Matrix<double, 1, Dynamic> CV;
	Simulation(int, int);

	void update();

	int getNumTris() { return 2 * (cloth.xRes - 1) * (cloth.yRes - 1); }
	int getNumPoints() { return cloth.xRes * cloth.yRes; }

};
