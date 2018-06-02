#include "UV.h"

UV::UV(Cloth &cloth, int *tri){


	auto t0 = Vector3d(cloth.getUvPoint(tri[0]));
	auto t1 = Vector3d(cloth.getUvPoint(tri[1]));
	auto t2 = Vector3d(cloth.getUvPoint(tri[2]));


	
	du1 = t1[0] - t0[0];
	dv1 = t1[1] - t0[1];
	du2 = t2[0] - t0[0];
	dv2 = t2[1] - t0[1];
	det = du1 * dv2 - du2 * dv1;
	UvArea=((t1 - t0).cross(t2 - t0)).norm()/2;
	alpha= sqrt(UvArea) * sqrt(sqrt(UvArea));

	x0 = Vector3d(cloth.getWorldPoint(tri[0]));
	x1 = Vector3d(cloth.getWorldPoint(tri[1]));
	x2 = Vector3d(cloth.getWorldPoint(tri[2]));

	wu = ((x1 - x0) * dv2 - (x2 - x0) * (dv1)) / det;
	wunorm = wu.norm();
	iwunorm = 1 / wunorm;
	whatu = wu * iwunorm;

	wv = (-(x1 - x0) * du2 + (x2 - x0) * du1) / det;
	wvnorm = wv.norm();
	iwvnorm = 1 / wvnorm;
	whatv = wv * iwvnorm;

	dwu_dxmx[0] = (dv1 - dv2) / det;
	dwu_dxmx[1] = dv2 / det;;
	dwu_dxmx[2] = -dv1 / det;
	dwv_dxmx[0] = (du2 - du1) / det;
	dwv_dxmx[1] = -du2 / det;
	dwv_dxmx[2] = du1 / det;
}
