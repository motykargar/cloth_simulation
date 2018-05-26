#pragma once
#include <Eigen/Dense>

#include "cloth.h"

#include "Cshear.h"
#include "Cstrech.h"

using namespace Eigen;


void dForcex(Cloth &, int *, double, double, double, double, double);


