#pragma once

#include <Eigen/Sparse>

#include <string.h>
#include <iostream>
#include <random>
#include <fstream>
#include <cstring>
#include <sstream>


#include "cloth.h"
#include "UV.h"
#include "W.h"
#include "Cshear.h"
#include "Cstrech.h"
#include "Bend.h"



using namespace Eigen;

#define STRETCHLIMIT  0.1
#define DENSITY 1
#define STRETCH_STIFF 5000
#define DAMP_STIFF  0.2
#define SHEAR_STIFF 500
#define BEND_STIFF 0.0001
#define IT 5
#define TIMESTEP 0.1
#define TOLERANCE  0.05 //1e-2f //0.1f
#define NB_INTERNAL_STEPS_INC_DEFAULT 2
#define MAX_NB_INTERNAL_STEPS_INC  40
#define INC_CHANGE_FACTOR  1.5f

#define FLOOR_HEIGHT -1

#define MAX_SCALE 0.000000001
#define MAX_BEND 0.000003

typedef Matrix<double, Dynamic, Dynamic> ForceMatrix;
typedef Matrix<double, 3, 3> dForceMatrix;
typedef Matrix<double, Dynamic, Dynamic> DForceMatrix;
typedef Matrix<double, Dynamic, Dynamic> PMatrix;
class Simulation {

	
	double scaleX;
	double scaleY;
	double h;
	int nbSubSteps;
	int maxSubSteps;
	int nbInternalStepsInc;
	int nbInternalSteps;
	bool stepFailed;
	bool fullStep;
	bool running = true;
	bool dofinal = true;
	bool bannerStyle = false;
	bool cuffing = false;
	bool doFinaleInPre;
	bool inStep;
	bool stepSuccessFlag;
	float stepEnd;
	double mas;
//	SparseMatrix<double> A;
	DForceMatrix A;
	DForceMatrix M;
	DForceMatrix S;
	DForceMatrix ri;
	ForceMatrix V;
	ForceMatrix b;
	ForceMatrix LM;
	DForceMatrix z0;
	DForceMatrix x;
	ForceMatrix y;
	ForceMatrix forcesave;
	ForceMatrix forces;
	DForceMatrix dforcesv;
	DForceMatrix dforcesx;
	SparseMatrix<double> P;
	SparseMatrix<double> Pinv;
	VectorXd bhat;
	VectorXd  r;
	VectorXd  c;
	Matrix3d I;
	double delta0;
	double deltaNew;
	Matrix<double, 5, 1> fenergy;  // stretch, shear, bend, gravity, drag.
	Matrix<double, 5, Dynamic> trienergy;
	double nbSteps;
	bool done;
	int size;
	double TIME;
	double framePeriod;
	int frameRate;
	double frame;

	void fShearStretch(int,int);
	void fShearStretchHelper(int *,int);

//	void EfShearStretchHelper(int *, int);
	void multiply(VectorXd, MatrixXd, VectorXd);
	

	

	void fBend(int);
	void bendHelper(int *);
	
	

	double *genTrisFromMesh();
	double *genNorms();
	double *genTriNorms();
	void copyPoint(double *, double *);

	void preSubSteps();
	void postSubStepsFinale();
	void postSubSteps();
	void preStepCG();
	void stepCG();
	void filterCompInPlace(VectorXd);
	void filterInPlace(VectorXd);
	ForceMatrix filter(ForceMatrix);
	ForceMatrix filterComp(ForceMatrix);
	void changeStep(int);
	void setFrameRate();


public:
	Cloth cloth;
	
	double *triVerts;
	double *norms;
	
	
	Simulation(int, int);
	bool doneSubSteps;
	void update();
	void update2();
	int getNumTris() { return 2 * (cloth.xRes - 1) * (cloth.yRes - 1); }
	int getNumPoints() { return cloth.xRes * cloth.yRes; }

};
