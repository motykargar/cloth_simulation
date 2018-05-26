#include "Force.h"

void Force(Cloth &cloth, int *tri, double b_u, double b_v,
	double STRETCH_STIFF, double DAMP_STIFF, double SHEAR_STIFF, int ID)
{
	Cstrech cstrech(cloth, tri, b_u, b_v, ID);
	Cshear cshear(cloth, tri);
	
	// Fstretch
	//cloth.Force[tri[0] * 3 + 0] += 1;
	for (int mm = 0; mm < 3; mm++) {
		Vector3d forcest = (-STRETCH_STIFF * (cstrech.dcu_dxm(mm, 0) *cstrech.cu
			+ cstrech.dcv_dxm(mm, 0) * cstrech.cv));
		Vector3d forcesh = (-SHEAR_STIFF * cshear.dc_dxm(mm, 0)* cshear.c);
		Vector3d dampst = (-DAMP_STIFF * (cstrech.dcu_dxm(mm, 0) *cstrech.dcu_dt
			+ cstrech.dcv_dxm(mm, 0) * cstrech.dcv_dt));
		Vector3d dampsh = (-DAMP_STIFF * cshear.dc_dxm(mm, 0) * cshear.dc_dt);
		for (int nn = 0; nn < 3; nn++) {
			
 		cloth.Force[(3*tri[mm]) + nn] += forcest[nn];
	cloth.Force[(3 * tri[mm]) + nn] += dampst[nn];
			cloth.Force[(3 * tri[mm]) + nn] += forcesh[nn];
cloth.Force[(3 * tri[mm]) + nn] += dampsh[nn];
		}
	}

	for (int m = 0; m < 3; ++m) for (int n = 0; n < 3; ++n) {

		// dFstetch

		cloth.dforcev[3 * tri[n], 3 * tri[m]] += -DAMP_STIFF * (
			(cstrech.dcu_dxm(m, 0)* cstrech.dcu_dxm(n, 0).transpose())
			+ (cstrech.dcv_dxm(m, 0)* cstrech.dcv_dxm(n, 0).transpose()));


		// dFshear

		cloth.dforcev[3 * tri[n], 3 * tri[m]] += -DAMP_STIFF *
			(cshear.dc_dxm(m, 0)* cshear.dc_dxm(n, 0).transpose());

	}
	Matrix3d I;
	I.setZero();
	I(0, 0) += 1;
	I(1, 1) += 1;
	I(2, 2) += 1;


	for (int m = 0; m < 3; ++m) for (int n = 0; n < 3; ++n) {

		// dFstetch
		cloth.dforcex[3 * tri[n], 3 * tri[m]] += -STRETCH_STIFF * (
			(cstrech.dcu_dxm(m, 0)* (cstrech.dcu_dxm(n, 0).transpose()))
			+ (cstrech.dcv_dxm(m, 0)* cstrech.dcv_dxm(n, 0).transpose())
			+ cstrech.d2cu_dxmdxn(m, n) * cstrech.cu
			+ cstrech.d2cv_dxmdxn(m, n) * cstrech.cv);


		cloth.dforcex[3 * tri[n], 3 * tri[m]] += -DAMP_STIFF * (
			cstrech.d2cu_dxmdxn(m, n) * cstrech.dcu_dt
			+ cstrech.d2cv_dxmdxn(m, n) * cstrech.dcv_dt);



		// dFshear

		cloth.dforcex[3 * tri[n], 3 * tri[m]] += -SHEAR_STIFF * (
			(cshear.dc_dxm(m, 0)* cshear.dc_dxm(n, 0).transpose())
			+ cshear.d2c_dxmdxn(m, n) * cshear.c*I);

		cloth.dforcex[3 * tri[n], 3 * tri[m]] += I * (-DAMP_STIFF * (
			cshear.d2c_dxmdxn(m, n) * cshear.dc_dt));

	}

	
}