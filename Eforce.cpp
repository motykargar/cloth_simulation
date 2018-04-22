#include "Eforce.h"

EFMatrix EForce(Cloth &cloth, int *tri, double b_u, double b_v,
	double STRETCH_STIFF, double DAMP_STIFF, double SHEAR_STIFF)
{
	Cstrech cstrech(cloth, tri, b_u, b_v);
	Cshear cshear(cloth, tri);
	EFMatrix R;
	

	//E stretch
	R(0,0) = STRETCH_STIFF * .5 * (cstrech.cu * cstrech.cu + cstrech.cv * cstrech.cv);
	

	R(1, 0) = SHEAR_STIFF * .5 * cshear.c * cshear.c;

	//E = .5 * k * bv._C * bv._C;
	
	return R;
}