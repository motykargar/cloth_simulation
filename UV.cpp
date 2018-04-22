#include "UV.h"

UV::UV(Cloth &cloth, int *tri){


	p0 = cloth.getUvPoint(tri[0]);
	p1 = cloth.getUvPoint(tri[1]);
	p2 = cloth.getUvPoint(tri[2]);
	t0 = { p0[0], p0[1], p0[2] };
	t1 = { p1[0], p1[1], p1[2] };
	t2 = { p2[0], p2[1], p2[2] };
	
	du1 = t1[0] - t0[0];
	dv1 = t1[1] - t0[1];
	du2 = t2[0] - t0[0];
	dv2 = t2[1] - t0[1];
	det = du1 * dv2 - du2 * dv1;
	UvArea=((t1 - t0).cross(t2 - t0)).norm()/2;
	alpha= sqrt(UvArea) * sqrt(sqrt(UvArea));
	
}
