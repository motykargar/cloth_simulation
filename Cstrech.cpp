#include "Cstrech.h"


Cstrech::Cstrech(Cloth &cloth, int *tri, double b_u, double b_v, int ID)
	: uv(UV(cloth, tri)), w(W(cloth, tri)) {
	uv = UV(cloth, tri);
	w = W(cloth, tri);
	double rho = 0.1;
	 v0 = Vector3d(cloth.getWorldVel(tri[0]));
	 v1 = Vector3d(cloth.getWorldVel(tri[1]));
	 v2 = Vector3d(cloth.getWorldVel(tri[2]));
	
	cu=uv.alpha * (w.wunorm - b_u);
	cv=uv.alpha * (w.wvnorm - b_v);
	for (int m = 0; m < 3; ++m)
	{
		dcu_dxm(m, 0) = (uv.alpha * w.dwu_dxmx[m]) * w.whatu;
		dcv_dxm(m, 0) = (uv.alpha * w.dwv_dxmx[m]) * w.whatv;
	}
	dcu_dt = 0;
	dcv_dt= 0;
	
	dcu_dt += dcu_dxm(0, 0).dot(v0);
	dcv_dt += dcv_dxm(0, 0).dot(v0);
	dcu_dt += dcu_dxm(1, 0).dot(v1);
	dcv_dt += dcv_dxm(1, 0).dot(v1);
	dcu_dt += dcu_dxm(2, 0).dot(v2);
	dcv_dt += dcv_dxm(2, 0).dot(v2);
	Matrix3d I;
	I.setZero();
	I(0, 0) += 1;
	I(1, 1) += 1;
	I(2, 2) += 1;
	
	for (int m = 0; m < 3; ++m) for (int n = m; n < 3; ++n)
		{
			d2cu_dxmdxn(m, n) = (uv.alpha * w.iwunorm * w.dwu_dxmx[m] * w.dwu_dxmx[n])
				* (I - (Matrix3d(w.whatu * w.whatu.transpose())));
			d2cu_dxmdxn(n, m) = d2cu_dxmdxn(m, n);
			d2cv_dxmdxn(m, n) = (uv.alpha * w.iwvnorm * w.dwv_dxmx[m] * w.dwv_dxmx[n])
				* (I - (Matrix3d(w.whatv * w.whatv.transpose())));
			d2cv_dxmdxn(n, m) = d2cv_dxmdxn(m, n);
		}

	
	cloth.Cu[ID] = cu/uv.alpha;
	cloth.Cv[ID] = cv/ uv.alpha;
	
	cloth.mass = rho * uv.UvArea / 3;
//td::cout << "cloth.Cu[ID]" << cloth.Cu[ID] << std::endl;
//td::cout << "doFinaleInPre4" << std::endl;
	
	
}