#pragma once
#include <Eigen/Dense>

#include "cloth.h"
#include "UV.h"
#include "W.h"
#include "Cshear.h"
#include "Cstrech.h"

using namespace Eigen;

typedef Matrix<Vector3d, 2, 3> FMatrix;


void Force(Cloth &, int *, double, double, double, double, double, int);

