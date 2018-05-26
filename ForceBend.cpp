#include "ForceBend.h"

void ForceBend(Cloth &cloth, int *tris, double DAMP_STIFF, double BEND_STIFF)
{
	Bend bend(cloth, tris);
	

	for (int m = 0; m < 4; m++) {
		Vector3d forcebend = -BEND_STIFF * bend.dc_dxm(m, 0) * bend.c;
		Vector3d dampbend = -DAMP_STIFF * bend.dc_dxm(m, 0) * bend.dc_dt;

		for (int nn = 0; nn < 3; nn++) {

			cloth.Force[(tris[m] * 3) + nn] += forcebend[nn];
			cloth.Force[(tris[m] * 3) + nn] += dampbend[nn];

		}

	}
}