#include "W.h"

W::W(Cloth &cloth, int *tri)
	: uv(UV(cloth, tri)) {
	uv = UV(cloth, tri);

	 p0 = cloth.getWorldPoint(tri[0]);
	 p1 = cloth.getWorldPoint(tri[1]);
	 p2 = cloth.getWorldPoint(tri[2]);
	t0 = { p0[0], p0[1], p0[2] };
	t1 = { p1[0], p1[1], p1[2] };
	t2 = { p2[0], p2[1], p2[2] };
	wu = ((t1 - t0) * uv.dv2 + (t2 - t0) * (uv.dv1)) / uv.det;
	wunorm = wu.norm();
	iwunorm = 1 / wunorm;
	whatu = wu * iwunorm;
	
	wv = (-(t1 - t0) * uv.du2 + (t2 - t0) * uv.du1) / uv.det;
	wvnorm = wv.norm();
	iwvnorm = 1 / wvnorm;
	whatv = wv * iwvnorm;

	dwu_dxmx[0]= (uv.dv1 - uv.dv2) /uv.det;
	dwu_dxmx[1] = uv.dv2 /uv.det;;
	dwu_dxmx[2] = -uv.dv1 /uv.det;
	dwv_dxmx[0]= (uv.du2 - uv.du1) /uv.det;
	dwv_dxmx[1] = -uv.du2 /uv.det;
	dwv_dxmx[2] =  uv.du1 /uv.det;
}
