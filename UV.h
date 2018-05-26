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

	UV(Cloth &, int *);
};
