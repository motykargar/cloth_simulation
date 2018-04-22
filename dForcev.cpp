#include "dForcev.h"

dFMatriv dForcev(Cloth &cloth, int *tri, double b_u, double b_v,
	double STRETCH_STIFF, double DAMP_STIFF, double SHEAR_STIFF)
{
	Cstrech cstrech(cloth, tri, b_u, b_v);
	Cshear cshear(cloth, tri);
	
	dFMatriv Rv;
	for (int m = 0; m < 3; m++) for (int n = 0; n < 3; n++) {
		Rv(n, m).setZero();
	}
	
	for (int m = 0; m < 3; ++m) for (int n = 0; n < 3; ++n) {

		// dFstetch
		
		Rv(n, m) += -DAMP_STIFF * (
			(cstrech.dcu_dxm(m, 0)* cstrech.dcu_dxm(n, 0).transpose())
			+ (cstrech.dcv_dxm(m, 0)* cstrech.dcv_dxm(n, 0).transpose()));


		// dFshear

		Rv(n, m) += -DAMP_STIFF *
			(cshear.dc_dxm(m, 0)* cshear.dc_dxm(n, 0).transpose());

	}

	return Rv;
}