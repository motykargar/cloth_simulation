#pragma once
#include <Eigen/Dense>

#include "cloth.h"

#include "Cshear.h"
#include "Cstrech.h"

using namespace Eigen;

typedef Matrix<double, 2, 1> EFMatrix; //E stretch  Eshear


EFMatrix EForce(Cloth &, int *, double, double, double, double, double);



