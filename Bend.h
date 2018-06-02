#pragma once


#include <Eigen/Dense>
using namespace Eigen;

#include "cloth.h"
#include "UV.h"



//pedef Matrix<Matrix3d, 3, 3> d2cMatrix;
typedef Matrix<Vector3d, 1, 4> qMatrix;
typedef Matrix<double, 1, 4> qeMatrix;
typedef Matrix<Vector3d, 4, 3> dnMatrix;
typedef Matrix<Vector3d, 3, 3> d2ntMatrix;
typedef Matrix<d2ntMatrix, 4, 4> d2nMatrix;
typedef Matrix<double, 4, 3> dsincosMatrix;
typedef Matrix<Vector3d, 4, 1> dcMatrix;
//typedef Matrix<Vector3d, 1, 3> ddcMatrix;
typedef Matrix<Matrix3d, 4, 4> d2cMatrix;


class Bend {

	
	Vector3d v0;
	Vector3d v1;
	Vector3d v2;
	Vector3d v3;
	Vector3d na;
	Vector3d nb;
	Vector3d e;
	double naim;
	double nbim;
	double eim;
	Vector3d nhata;
	Vector3d nhatb;
	Vector3d ehat;
	double sint;
	double cost;
	
	qMatrix qa;
	qMatrix qb;
	Vector4d qe;
	dnMatrix dnhatA_dmx;
	dnMatrix dnhatB_dmx;
	dnMatrix dehat_dmx;
	d2nMatrix d2nA_dxmdxn;
	d2nMatrix d2nB_dxmdxn;
	dsincosMatrix dcos_dxm;
	dsincosMatrix dsin_dxm;
	d2cMatrix d2cos;
	d2cMatrix d2sin;
	



	void N(Cloth &, int *);
	void E(Cloth &, int *);
	void C();
	void Qs(Cloth &, int *);
	void NEderiv();
	void D2NE();
	void dsincos();
	void d2sincos();
	void cderiv();

public:
	double c;
	dcMatrix dc_dxm;
	d2cMatrix d2c_dxmdxn;
	double dc_dt;

	Matrix3d makeSkew(Vector3d);
	
	Bend(Cloth &, int *);


};
