#include "Bend.h"


Bend::Bend(Cloth &cloth, int *tri) {
	
	N(cloth, tri);
	E(cloth, tri);
	C();
	Qs(cloth, tri);
	NEderiv();
	D2NE();
	dsincos();
	d2sincos();
	cderiv();
	p0 = cloth.getWorldVel(tri[0]);
	p1 = cloth.getWorldVel(tri[1]);
	p2 = cloth.getWorldVel(tri[2]);
	p3 = cloth.getWorldVel(tri[3]);
	v0 = { p0[0], p0[1], p0[2] };
	v1 = { p1[0], p1[1], p1[2] };
	v2 = { p2[0], p2[1], p2[2] };
	v3 = { p3[0], p3[1], p3[2] };
	dc_dt = 0;

	dc_dt += (dc_dxm(0,0)).dot(v0);
	dc_dt += (dc_dxm(1, 0)).dot(v1);
	dc_dt += (dc_dxm(2, 0)).dot(v2);
	dc_dt += (dc_dxm(3, 0)).dot(v3);
}


void Bend::N(Cloth &cloth, int *tri) {

	na= cloth.triNormal(tri[0], tri[1], tri[2]);
	nb= cloth.triNormal(tri[2], tri[1], tri[3]);
	
	naim = 1 / na.norm();
	nbim = 1 / nb.norm();
	
	nhata= na * naim;
	nhatb = nb * nbim;
}

void Bend::E(Cloth &cloth, int *tri) {
	
	auto x1 = Vector3d(cloth.getWorldPoint(tri[1]));
	auto x2 = Vector3d(cloth.getWorldPoint(tri[2]));
	e = x1 - x2;
	eim = 1 / e.norm();

	ehat = e * eim;
}

void Bend::C() {

	sint = nhata.cross(nhatb).dot(ehat);
	cost = nhata.dot(nhatb);
	
	c = atan2(sint, cost);
}

void Bend::Qs(Cloth &cloth, int *tri){

	auto x0 = Vector3d(cloth.getWorldPoint(tri[0]));
	auto x1 = Vector3d(cloth.getWorldPoint(tri[1]));
	auto x2 = Vector3d(cloth.getWorldPoint(tri[2]));
	auto x3 = Vector3d(cloth.getWorldPoint(tri[3]));
	Vector3d z = { 0,0,0 };

	qa(0, 0) = x2 - x1;
	qa(0, 1) = x0 - x2;
	qa(0, 2) = x1 - x0;
	qa(0, 3) = z;

	qb(0, 0) = z;
	qb(0, 1) = x2 - x3;
	qb(0, 2) = x3 - x1;
	qb(0, 3) = x1 - x2;

	qe[0] = 0;
	qe[1] = 1;
	qe[2] = -1;
	qe[3] = 0;


}

void Bend::NEderiv() {
	Matrix3d I;
	I.setZero();
	I(0, 0) += 1;
	I(1, 1) += 1;
	I(2, 2) += 1;

	for (int m = 0; m < 4; ++m) {
		for (int s = 0; s < 3; ++s) {
			dnhatA_dmx(m, s) = (makeSkew(qa(0,m)).row(s))*naim;
			 dnhatB_dmx(m,s)= (makeSkew(qb(0, m)).row(s))*nbim;
			 dehat_dmx(m, s) = (I).col(s) *qe[m] * eim;
		}
	}

}

void Bend::D2NE() {
	
	Matrix<Vector4d, 1, 4> D1;
	Matrix<Vector4d, 1, 4> D2;
	/////////////???? why -1 Instead 1????
	D1 = {{ 0, -1, 1, 0 },
	{ 1, 0, -1, 0 },
	{ -1, 1, 0, 0 },
	{ 0, 0, 0, 0 }
	};
	D2 = {{ 0, 0, 0, 0 },
	{ 0, 0, 1, -1 },
	{ 0, -1, 0, 1 },
	{ 0, 1, -1, 0 }
	};
	Matrix3d I;
	I.setZero();
	I(0, 0) += 1;
	I(1, 1) += 1;
	I(2, 2) += 1;

	for (int m = 0; m < 4; ++m) {

		for (int n = 0; n < 4; ++n) {
			Matrix3d dqAm_dxn_t = I*D1(0,m)[n] * naim;
			Matrix3d dqBm_dxn_t = I*D2(0,m)[n] * nbim;

			for (int t = 0; t < 3; ++t) {
				Matrix3d sqat = makeSkew(dqAm_dxn_t.col(t));
				Matrix3d sqbt = makeSkew(dqBm_dxn_t.col(t));
				for (int s = 0; s < 3; ++s) {
					
					d2nA_dxmdxn(m, n)(s, 0) = Vector3d(sqat.row(s));
					d2nB_dxmdxn(m, n)(s, 0) = Vector3d(sqbt.row(s));
					
				}
			}
		}
	}

}

void Bend::dsincos() {
	for (int m = 0; m < 4; ++m) {
		for (int s = 0; s < 3; ++s) {
			dcos_dxm(m,s)=
				dnhatA_dmx(m, s).dot(nhatb) +
				nhata.dot(dnhatB_dmx(m, s));
		}
	}
	for (int m = 0; m < 4; ++m) {
		for (int s = 0; s < 3; ++s) {
			dsin_dxm(m, s) =
				(dnhatA_dmx(m, s).cross(nhatb) +
					nhata.cross(dnhatB_dmx(m, s))).dot(ehat)
				+ (nhata.cross(nhatb)).dot(dehat_dmx(m, s));
		}
	}

}

void Bend::d2sincos() {
	for (int m = 0; m < 4; ++m) {
		for (int n = 0; n < 4; ++n) {
		

			for (int s = 0; s < 3; ++s){
				for (int t = 0; t < 3; ++t) {
					
					(d2cos(m, n)(s, 0))[t]=
					(d2nA_dxmdxn(m, n)(s, 0)).dot(nhatb) +
						dnhatA_dmx(m, s).dot(dnhatB_dmx(n, t)) +
						dnhatA_dmx(n, t).dot(dnhatB_dmx(m, s)) +
						(d2nB_dxmdxn(m, n)(s, 0)).dot(nhata);


					(d2sin(m, n)(s, 0))[t] =
						((d2nA_dxmdxn(m, n)(s, 0)).cross(nhatb) +
						dnhatA_dmx(m, s).cross(dnhatB_dmx(n, t)) +
						dnhatA_dmx(n, t).cross(dnhatB_dmx(m, s)) +
						nhata.cross((d2nB_dxmdxn(m, n)(s, 0)))).dot(ehat) +
						(dnhatA_dmx(m, s).cross(nhatb) + nhata.cross(dnhatB_dmx(m, s))).dot(dehat_dmx(n, t))
						+ (dnhatA_dmx(n, t).cross(nhatb) + nhata.cross(dnhatB_dmx(n, t))).dot(dehat_dmx(m, s));
				}
			}
		}
	}

}

void Bend::cderiv() {
	for (int m = 0; m < 4; ++m) {
		dc_dxm(m,0)= cost * dsin_dxm.row(m)	-sint * dcos_dxm.row(m);
	}

	for (int m = 0; m < 4; ++m) {
		
		for (int n = m; n < 4; ++n) {
			
			for (int s = 0; s < 3; ++s) for (int t = 0; t < 3; ++t) {
				double a =cost * (d2sin(m, n)(t, 0))[s]
					- sint * (d2cos(m, n)(t, 0))[s];
				double b = dcos_dxm(n, t) * dsin_dxm(m, s)
					- dsin_dxm(n, t) * dcos_dxm(m, s);
				(d2c_dxmdxn(m, n)(t, s)) = a + b;
				(d2c_dxmdxn(n, m)(s, t)) = a - b;

			}
		}
	}
}








Matrix3d Bend::makeSkew(Vector3d q) {

	Matrix<double, 3, 3> D;
	D(0, 0) = 0; D(0, 1) = -q[2]; D(0, 2) = q[1];
	D(1, 0) = q[2]; D(1, 1) = 0; D(1, 2) = -q[0];
	D(2, 0) = -q[1]; D(2, 1) = q[0]; D(2, 2) = 0;

	return D;
}


