#pragma once

#include <string.h>
#include <iostream>

#include <Eigen/Dense>
using namespace Eigen;

#include "cloth.h"

struct UV {

	
public:
	double du1;
	double dv1;
	double du2;
	double dv2; 
	double det;
	double UvArea;
	double alpha;   //David Pritchard
	Vector3d x0;
	Vector3d x1;
	Vector3d x2;
	
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

	UV(Cloth &, int *);
};
