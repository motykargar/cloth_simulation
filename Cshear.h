#pragma once

#include <Eigen/Dense>
using namespace Eigen;

#include "cloth.h"
#include "UV.h"


typedef Matrix<Vector3d, 3, 1> dcSHMatrix;
//pedef Matrix<double, 3, 3> d2cMatrix;
struct Cshear {
	Vector3d v0;
	Vector3d v1;
	Vector3d v2;
public:
	UV uv;
	
	double c;
	
	dcSHMatrix dc_dxm;
	double dc_dt;
	Matrix3d d2c_dxmdxn;

	Cshear(Cloth &, int *);


};

