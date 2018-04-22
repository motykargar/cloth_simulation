#include "ForceBend.h"

FBendMatrix ForceBend(Cloth &cloth, int *tris, double DAMP_STIFF, double BEND_STIFF)
{
	Bend bend(cloth, tris);
	FBendMatrix R;


	for (int m = 0; m < 4; ++m) {
		
		R(0, m) = -BEND_STIFF * bend.dc_dxm(m, 0) * bend.c;
		

		R(0, m) += -DAMP_STIFF * bend.dc_dxm(m, 0) * bend.dc_dt;
		
	}

	
	return R;
}