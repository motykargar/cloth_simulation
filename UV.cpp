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
	
}
