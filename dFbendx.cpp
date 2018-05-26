#include "dFbendx.h"

void dFbendx(Cloth &cloth, int *tris, double DAMP_STIFF, double BEND_STIFF)
{
	Bend bend(cloth, tris);

	for (int m = 0; m < 4; ++m) for (int n = 0; n < 4; ++n) {
		
		cloth.dforcex[tris[n]*3, tris[m]*3] += -BEND_STIFF * ((bend.dc_dxm(m, 0)* bend.dc_dxm(n, 0).transpose())
			+ bend.d2c_dxmdxn(m, n) * bend.c);
			
		  cloth.dforcex[tris[n]*3, tris[m]*3] += -DAMP_STIFF * bend.d2c_dxmdxn(m, n) * bend.dc_dt;

	}
}