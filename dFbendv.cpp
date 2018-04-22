#include "dFbendv.h"

dFbendMatrix dFbendv(Cloth &cloth, int *tris, double DAMP_STIFF, double BEND_STIFF)
{
	Bend bend(cloth, tris);

	dFbendMatrix Rv;

	for (int m = 0; m < 4; m++) for (int n = 0; n < 4; n++) {
		Rv(n, m).setZero();
	}



	for (int m = 0; m < 4; ++m) for (int n = 0; n < 4; ++n) {


		Rv(m, n) += -DAMP_STIFF * (bend.dc_dxm(m, 0)* bend.dc_dxm(n, 0).transpose());



	}



	return Rv;
}