#pragma once
#include <Eigen/Dense>

#include "cloth.h"
#include "UV.h"
#include "W.h"
#include "Bend.h"


using namespace Eigen;

typedef Matrix<Vector3d, 1, 4> FBendMatrix;


FBendMatrix ForceBend(Cloth &, int *, double, double);


