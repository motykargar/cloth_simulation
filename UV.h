#pragma once


#include <Eigen/Dense>
using namespace Eigen;

#include "cloth.h"

struct UV {

	double *p0;
	double *p1;
	double *p2;
	Vector3d t0;
	Vector3d t1;
	Vector3d t2;
	double du1;
	double dv1;
	double du2;
	double dv2; 
	double det;
	double UvArea;
	double alpha;   //David Pritchard

	UV(Cloth &, int *);
	

};
