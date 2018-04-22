#include "Cshear.h"


Cshear::Cshear(Cloth &cloth, int *tri)
	: uv(UV(cloth, tri)), w(W(cloth, tri)) {
	uv = UV(cloth, tri);
	w = W(cloth, tri);


	p0 = cloth.getWorldVel(tri[0]);
	p1 = cloth.getWorldVel(tri[1]);
	p2 = cloth.getWorldVel(tri[2]);
	v0 = { p0[0], p0[1], p0[2] };
	v1 = { p1[0], p1[1], p1[2] };
	v2 = { p2[0], p2[1], p2[2] };

	//c = uv.alpha * ((w.whatu).dot(w.whatv));
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
	

	//Matrix3d::indent
	for (int m = 0; m < 3; ++m) for (int n = m; n < 3; ++n)
	{
		d2c_dxmdxn(m, n) =  uv.alpha * (
				w.dwu_dxmx[m] * w.dwv_dxmx[n] +
				w.dwu_dxmx[n] * w.dwv_dxmx[m]);
		d2c_dxmdxn(n, m) = uv.alpha * (
			w.dwu_dxmx[m] * w.dwv_dxmx[n] +
			w.dwu_dxmx[n] * w.dwv_dxmx[m]);
	}
}