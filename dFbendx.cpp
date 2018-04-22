#include "dFbendx.h"

dFbendMatrix dFbendx(Cloth &cloth, int *tris, double DAMP_STIFF, double BEND_STIFF)
{
	Bend bend(cloth, tris);
	
	dFbendMatrix Rx;

	for (int m = 0; m < 4; m++) for (int n = 0; n < 4; n++) {
		Rx(n, m).setZero();
	}



	for (int m = 0; m < 4; ++m) for (int n = 0; n < 4; ++n) {

		// dFstetch
		Rx(n, m) += -BEND_STIFF * ((bend.dc_dxm(m, 0)* bend.dc_dxm(n, 0).transpose())
			+ bend.d2c_dxmdxn(m, n) * bend.c);
			//Rx(m, n) += -BEND_STIFF * ((bend.dc_dxm(m, 0)* bend.dc_dxm(n, 0).transpose()));

		Rx(n, m) += -DAMP_STIFF * bend.d2c_dxmdxn(m, n) * bend.dc_dt;



	}



	return Rx;
}