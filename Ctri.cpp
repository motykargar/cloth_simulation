#include "Ctri.h"

EFMatrix Ctri(Cloth &cloth, int *tri, double b_u, double b_v,
	double STRETCH_STIFF, double DAMP_STIFF, double SHEAR_STIFF)
{
	Cstrech cstrech(cloth, tri, b_u, b_v);
	UV uv(cloth, tri);
	EFMatrix R;


	R(0, 0) = cstrech.cu / uv.alpha;
	R(1, 0) = cstrech.cv / uv.alpha;
	

	return R;
}