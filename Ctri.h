#pragma once
#include <Eigen/Dense>

#include "cloth.h"
#include "UV.h"
#include "Cstrech.h"

using namespace Eigen;

typedef Matrix<double, 2, 1> EFMatrix; //CU CV


EFMatrix Ctri(Cloth &, int *, double, double, double, double, double);




