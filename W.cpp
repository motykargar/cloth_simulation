#include "W.h"

W::W(Cloth &cloth, int *tri)
	: uv(UV(cloth, tri)) {
	uv = UV(cloth, tri);

	 x0 = Vector3d(cloth.getWorldPoint(tri[0]));
	 x1 = Vector3d(cloth.getWorldPoint(tri[1]));
	 x2 = Vector3d(cloth.getWorldPoint(tri[2]));
	
	wu = ((x1 - x0) * uv.dv2 - (x2 - x0) * (uv.dv1)) / uv.det;
	wunorm = wu.norm();
	iwunorm = 1 / wunorm;
	whatu = wu * iwunorm;
	
	wv = (-(x1 - x0) * uv.du2 + (x2 - x0) * uv.du1) / uv.det;
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
