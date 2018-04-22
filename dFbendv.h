#pragma once
#include <Eigen/Dense>

#include "cloth.h"

#include "Bend.h"


using namespace Eigen;

typedef Matrix<Matrix3d, 4, 4> dFbendMatrix;


dFbendMatrix dFbendv(Cloth &, int *, double, double);



