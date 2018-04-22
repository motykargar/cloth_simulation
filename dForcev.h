#pragma once
#include <Eigen/Dense>

#include "cloth.h"

#include "Cshear.h"
#include "Cstrech.h"

using namespace Eigen;

typedef Matrix<Matrix3d, 3, 3> dFMatriv;


dFMatriv dForcev(Cloth &, int *, double, double, double, double, double);



