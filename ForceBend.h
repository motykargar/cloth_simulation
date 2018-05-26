#pragma once
#include <Eigen/Dense>

#include "cloth.h"
#include "UV.h"
#include "W.h"
#include "Bend.h"


using namespace Eigen;


void ForceBend(Cloth &, int *, double, double);


