#include "Cshear.h"


Cshear::Cshear(Cloth &cloth, int *tri)
	: uv(UV(cloth, tri)), w(W(cloth, tri)) {
	uv = UV(cloth, tri);
	w = W(cloth, tri);

	 v0 = Vector3d(cloth.getWorldVel(tri[0]));
	 v1 = Vector3d(cloth.getWorldVel(tri[1]));
	 v2 = Vector3d(cloth.getWorldVel(tri[2]));
	c = uv.alpha * ((w.wu).dot(w.wv));
	
	for (int m = 0; m < 3; ++m) {
		for (int s = 0; s < 3; ++s) {
			
				dc_dxm(m, 0)[s] = uv.alpha * (w.dwu_dxmx[m] * w.wv[s] +
					w.wu[s] * w.dwv_dxmx[m]);
		}
	}

	dc_dt =0;

	dc_dt += (dc_dxm(0, 0)).dot(v0);
	dc_dt += (dc_dxm(1, 0)).dot(v1);
	dc_dt += (dc_dxm(2, 0)).dot(v2);
	

	for (int m = 0; m < 3; ++m) for (int n = m; n < 3; ++n)
	{
		d2c_dxmdxn(m, n) =  uv.alpha * (
				w.dwu_dxmx[m] * w.dwv_dxmx[n] +
				w.dwu_dxmx[n] * w.dwv_dxmx[m]);
		d2c_dxmdxn(n, m) = d2c_dxmdxn(m, n);
	}
}