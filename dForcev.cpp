#include "dForcev.h"

void dForcev(Cloth &cloth, int *tri, double b_u, double b_v,
	double STRETCH_STIFF, double DAMP_STIFF, double SHEAR_STIFF)
{
	Cstrech cstrech(cloth, tri, b_u, b_v);
	Cshear cshear(cloth, tri);
	
	
	
	for (int m = 0; m < 3; ++m) for (int n = 0; n < 3; ++n) {

		// dFstetch
		
		cloth.dforcev[tri[m], tri[n]] += -DAMP_STIFF * (
			(cstrech.dcu_dxm(m, 0)* cstrech.dcu_dxm(n, 0).transpose())
			+ (cstrech.dcv_dxm(m, 0)* cstrech.dcv_dxm(n, 0).transpose()));


		// dFshear

		cloth.dforcev[tri[m], tri[n]] += -DAMP_STIFF *
			(cshear.dc_dxm(m, 0)* cshear.dc_dxm(n, 0).transpose());

	}

}