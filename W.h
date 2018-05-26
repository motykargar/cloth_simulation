#pragma once

#include <Eigen/Dense>
using namespace Eigen;

#include "cloth.h"
#include "UV.h"

struct W {


public: 
	Vector3d x0;
	Vector3d x1;
	Vector3d x2;
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
