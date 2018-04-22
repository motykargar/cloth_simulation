#pragma once

#include <Eigen/Dense>
using namespace Eigen;

#include "cloth.h"
#include "UV.h"

struct W {

	double *p0;
	double *p1;
	double *p2;
	Vector3d t0;
	Vector3d t1;
	Vector3d t2;

public:
	UV uv;
	Vector3d wu;
	Vector3d wv;
	double wunorm;
	double wvnorm; 
	double iwunorm;
	double iwvnorm;
	Vector3d whatu;
	Vector3d whatv;
	Vector3d dwu_dxmx;
	Vector3d dwv_dxmx;

	W(Cloth &, int *);


};
