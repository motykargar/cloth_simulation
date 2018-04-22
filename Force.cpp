#include "Force.h"

FMatrix Force(Cloth &cloth, int *tri, double b_u, double b_v,
	double STRETCH_STIFF, double DAMP_STIFF, double SHEAR_STIFF)
{
	Cstrech cstrech(cloth, tri, b_u, b_v);
	Cshear cshear(cloth, tri);
	FMatrix R;
	// Fstretch
	R(0, 0) = (-STRETCH_STIFF * (cstrech.dcu_dxm(0, 0) *cstrech.cu
		+ cstrech.dcv_dxm(0, 0) * cstrech.cv));
	R(0, 1) = (-STRETCH_STIFF * (cstrech.dcu_dxm(1, 0) *cstrech.cu
		+ cstrech.dcv_dxm(1, 0) * cstrech.cv));
	R(0, 2) = (-STRETCH_STIFF * (cstrech.dcu_dxm(2, 0) *cstrech.cu
		+ cstrech.dcv_dxm(2, 0) * cstrech.cv));
	R(0, 0) += (-DAMP_STIFF * (cstrech.dcu_dxm(0, 0) *cstrech.dcu_dt
		+ cstrech.dcv_dxm(0, 0) * cstrech.dcv_dt));
	R(0, 1) += (-DAMP_STIFF * (cstrech.dcu_dxm(1, 0) *cstrech.dcu_dt
		+ cstrech.dcv_dxm(1, 0) * cstrech.dcv_dt));
	R(0, 2) += (-DAMP_STIFF * (cstrech.dcu_dxm(2, 0) *cstrech.dcu_dt
		+ cstrech.dcv_dxm(2, 0) * cstrech.dcv_dt));
	// Fshear
	R(1, 0) = (-SHEAR_STIFF * cshear.dc_dxm(0, 0)* cshear.c);
	R(1, 1) = (-SHEAR_STIFF * cshear.dc_dxm(1, 0)* cshear.c);
	R(1, 2) = (-SHEAR_STIFF * cshear.dc_dxm(2, 0)* cshear.c);
	R(1, 0) += (-DAMP_STIFF * cshear.dc_dxm(0, 0) * cshear.dc_dt);
	R(1, 1) += (-DAMP_STIFF * cshear.dc_dxm(1, 0) * cshear.dc_dt);
	R(1, 2) += (-DAMP_STIFF * cshear.dc_dxm(2, 0) * cshear.dc_dt);
	
	
	return R;
}