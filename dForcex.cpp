#include "dForcex.h"

dFMatrix dForcex(Cloth &cloth, int *tri, double b_u, double b_v,
	double STRETCH_STIFF, double DAMP_STIFF, double SHEAR_STIFF)
{
	Cstrech cstrech(cloth, tri, b_u, b_v);
	Cshear cshear(cloth, tri);
	dFMatrix Rx;
	for (int m = 0; m < 3; m++) for (int n = 0; n < 3; n++) {
		Rx(n, m).setZero();
	}
	
	
	Matrix3d I;
	I.setZero();
	I(0, 0) += 1;
	I(1, 1) += 1;
	I(2, 2) += 1;
	

	for (int m = 0; m < 3; ++m) for (int n = 0; n < 3; ++n) {
		
		// dFstetch
		Rx(n,m) += -STRETCH_STIFF * (
			(cstrech.dcu_dxm(m, 0)* (cstrech.dcu_dxm(n, 0).transpose()))
			+ (cstrech.dcv_dxm(m, 0)* cstrech.dcv_dxm(n, 0).transpose())
			+ cstrech.d2cu_dxmdxn(m, n) * cstrech.cu
			+ cstrech.d2cv_dxmdxn(m, n) * cstrech.cv);
		

		Rx(n, m) += -DAMP_STIFF * (
			cstrech.d2cu_dxmdxn(m, n) * cstrech.dcu_dt
			+ cstrech.d2cv_dxmdxn(m, n) * cstrech.dcv_dt);



		// dFshear

		Rx(n, m) += -SHEAR_STIFF * (
			(cshear.dc_dxm(m, 0)* cshear.dc_dxm(n, 0).transpose())
						+ cshear.d2c_dxmdxn(m, n) * cshear.c*I);

		Rx(n, m) += I*( -DAMP_STIFF * (
			cshear.d2c_dxmdxn(m, n) * cshear.dc_dt));

	}


	

	return Rx;
}