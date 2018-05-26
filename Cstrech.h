#pragma once

#include <Eigen/Dense>
#include <string.h>
#include <iostream>
using namespace Eigen;

#include "cloth.h"
#include "UV.h"
#include "W.h"
typedef Matrix<Vector3d, 3, 1> dcSTMatrix;
typedef Matrix<Matrix3d, 3, 3> d2cSTMatrix;
struct Cstrech {

	Vector3d v0;
	Vector3d v1;
	Vector3d v2;
public:
	UV uv;
	W w;
	double cu;
	double cv;
	dcSTMatrix dcu_dxm;
	dcSTMatrix dcv_dxm;
	double dcu_dt;
	double dcv_dt;
	d2cSTMatrix d2cu_dxmdxn;
	d2cSTMatrix d2cv_dxmdxn;
	
	Cstrech(Cloth &, int *, double, double, int);


};
