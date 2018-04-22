#include "simulation.h"

using namespace std;
Simulation::Simulation(int clothXRes, int clothYRes)
	: cloth(Cloth(clothXRes, clothYRes)) {
	cloth = Cloth(cloth.xRes, cloth.yRes);
	maxScale = MAX_SCALE * 30 * 30 / (cloth.xRes * cloth.yRes);
	int h = 0;
	int trii = 0;
	int triii = 0;
	// regenerate triangles from the mesh
	triVerts = genTrisFromMesh();
	norms = genTriNorms();
	////std::cout << getNumTris() << std::endl;
	scaleX = 1;
	scaleY = 1;
	
	//forces0.resize(1, getNumPoints());
	forces.resize(3 * getNumPoints(), 1);
	dforcesx.resize(3 * getNumPoints(), 3 * getNumPoints());
	dforcesv.resize(3 * getNumPoints(), 3 * getNumPoints());
	M.resize(3 * getNumPoints(), 3 * getNumPoints());
	a.resize(3 * getNumPoints(), 3 * getNumPoints());
	LM.resize(3 * getNumPoints(), 3 * getNumPoints()+1);
	V.resize(3 * getNumPoints(), 1);
	b.resize(3 * getNumPoints(), 1);
	deltaV.resize(3 * getNumPoints(), 1);
	//Matrix<double, 5, 1> fenergy;  // stretch, shear, bend, gravity, drag.
	trienergy.resize(5, getNumTris());
	CU.resize(1, getNumTris());
	CV.resize(1, getNumTris());
	fenergy.setZero();
	trienergy.setZero();
	CU.setZero();
	CV.setZero();
   forces.setZero();
   dforcesx.setZero();
   dforcesv.setZero();
   M.setZero();
   a.setZero();
   LM.setZero();
   V.setZero();
   b.setZero();
   deltaV.setZero();
   ///// setupMass
	Matrix<double, 3, 3> mm;
	mm.setZero();
	 mas = DENSITY * cloth.triUvArea / 3;
	mm(0, 0) += 1/mas; mm(1, 1) += 1/mas;  mm(2, 2) += 1/mas;
	
	for (int pt = 0; pt < cloth.xRes * cloth.yRes; pt++) {
		for (int p = 0; p < cloth.xRes * cloth.yRes; p++) {
			for (int m = 0; m < 3; m++) for (int n = 0; n < 3; n++) {
				M((pt * 3) + m, (p * 3) + n) = mm(m, n);
			}
		}
	}
///	/////removeAllConstraints
	S.resize(3 * getNumPoints(), 3);
	S.setZero();
	z0.resize(3 * getNumPoints(), 1);
	z0.setZero();
	Matrix<double, 3, 3> SS;
	SS.setZero();
	SS(0, 0) += 1; SS(1, 1) += 1;  SS(2, 2) += 1;

for (int pt = 0; pt < cloth.xRes * cloth.yRes; pt++) {
			for (int m = 0; m < 3; m++) for (int n = 0; n < 3; n++) {
				S((pt * 3) + m, n) = SS(m, n);
		}
	}

}

void Simulation::update() {
	// if simulation is paused, don't update
	h += 1;
	trii = -1;
	triii = -1;
	
	if (h > IT) running = false;
	if (!running) return;
	//if (h > 1) trii -= getNumTris();

	// zero forces matrix
	
	fenergy.setZero();
	trienergy.setZero();
	CU.setZero();
	CV.setZero();
	forces.setZero();
	dforcesx.setZero();
	dforcesv.setZero();
	M.setZero();
	a.setZero();
	LM.setZero();
	V.setZero();
	b.setZero();
	deltaV.setZero();
	

	for (int i = 0; i < cloth.yRes - 1; i++) {
		for (int j = 0; j < cloth.xRes - 1; j++) {

			int offset = i * cloth.xRes + j;
			
			fShearStretch(offset);
			
			//fBend (offset);
		}
	}
	for (int i = 0; i < cloth.xRes * cloth.yRes; i++) {
		// update velocities by condition-forces
		
		forces((i * 3) + 1, 0) -= 9.8 *mas;

	}
	for (int i = 0; i < cloth.xRes * cloth.yRes; i++) {

		V((i * 3) + 0, 0) = cloth.worldVels[i * 3 + 0];
		V((i * 3) + 1, 0) = cloth.worldVels[i * 3 + 1];
		V((i * 3) + 2, 0) = cloth.worldVels[i * 3 + 2];
		

	}
		// Paper eq. (16)
	
	
			a = dforcesx * (-TIMESTEP * TIMESTEP) + dforcesv * (-TIMESTEP) + M;
			b = TIMESTEP * (TIMESTEP * dforcesx * V + forces);

			SparseMatrix<double> A(3 * cloth.xRes * cloth.yRes, 3 * cloth.xRes * cloth.yRes);
			A = a.sparseView();
			VectorXd deltaV(3 * cloth.xRes * cloth.yRes);
			VectorXd B(3 * cloth.xRes * cloth.yRes);
			B = b;
			double TOLERANCE = 1e-2f;
			/// These are wrong.
			ConjugateGradient<SparseMatrix<double>, Lower | Upper> cg;
			cg.setTolerance(TOLERANCE);
			cg.compute(A);
			deltaV = cg.solve(B);
			///////
			//std::cout << forces << std::endl;
			 for (int i = 0; i < cloth.xRes * cloth.yRes; i++) {

				cloth.worldVels[i * 3 + 0] += deltaV[(i * 3) + 0];
				 cloth.worldVels[i * 3 + 1] += deltaV[(i * 3) + 1];
				 cloth.worldVels[i * 3 + 2] += deltaV[(i * 3) + 2];
			

			 }
			
		
	// lock the top row of points, if we've enabled that setting
	int lastPoint = 3 * cloth.xRes * cloth.yRes;
	if (LOCK_TOP_ROW) lastPoint -= 3 * cloth.xRes;
	//if (LOCK_TOP_ROW) lastPoint -= 3 * 1;

	// move the points by their velocities
	for (int i = 0; i < lastPoint; i++) {
		
		cloth.worldPoints[i] += TIMESTEP * cloth.worldVels[i];
		std::cout << cloth.worldPoints[i] << std::endl;
	}

	// generate new triangles from the mesh
	if (triVerts) delete triVerts;
	if (norms) delete norms;
	triVerts = genTrisFromMesh();
	norms = genTriNorms();
}




void Simulation::fShearStretch(int offset) {
	int botLeftTri[3] = { offset, offset + 1, offset + cloth.xRes };
	int topRightTri[3] = { offset + cloth.xRes, offset + 1,
		offset + cloth.xRes + 1 };

	int x = offset % cloth.xRes;
	int y = offset / cloth.xRes;

	fShearStretchHelper(botLeftTri);
	fShearStretchHelper(topRightTri);
	dfShearStretchHelper(botLeftTri);
	dfShearStretchHelper(topRightTri);
	trii += 1;
	EfShearStretchHelper(botLeftTri);
	CuCv(botLeftTri);

	std::cout << trii << std::endl;
	trii += 1;
	std::cout << trii << std::endl;
	EfShearStretchHelper(topRightTri);
	CuCv(topRightTri);
}

void Simulation::fShearStretchHelper(int *triPts) {
	FMatrix	R = Force(cloth, triPts, scaleX, scaleY, STRETCH_STIFF, DAMP_STIFF, SHEAR_STIFF);

	//F STRETCH
	for (int g = 0; g < 3; g++) {
		forces(triPts[0] * 3 + g , 0) += R(0, 0)[g];
		forces(triPts[1] * 3 + g , 0) += R(0, 1)[g];
		forces(triPts[2] * 3 + g , 0) += R(0, 2)[g];
		//F SHEAR
		forces(triPts[0] * 3 + g , 0) += R(1, 0)[g];
		forces(triPts[1] * 3 + g , 0) += R(1, 1)[g];
		forces(triPts[2] * 3 + g , 0) += R(1, 2)[g];
	}
}

void Simulation::dfShearStretchHelper(int *triPts) {
	
	Matrix<Matrix3d, 3, 3>	Rx = dForcex(cloth, triPts, scaleX, scaleY, STRETCH_STIFF, DAMP_STIFF, SHEAR_STIFF);
	Matrix<Matrix3d, 3, 3> Rv = dForcev(cloth, triPts, scaleX, scaleY, STRETCH_STIFF, DAMP_STIFF, SHEAR_STIFF);

	for (int m = 0; m < 3; ++m) for (int n = 0; n < 3; ++n) {
		for (int g = 0; g < 3; g++)for (int c = 0; c < 3; c++) {

		dforcesx(triPts[m]*3+g, triPts[n]*3+c) += Rx(m, n)(g,c);
			dforcesv(triPts[m]*3+g, triPts[n]*3+c) += Rv(m, n)(g, c);
		}
	}
}


void Simulation::EfShearStretchHelper(int *triPts) {
	
	Matrix<double, 2, 1> Re;
		Re= EForce(cloth, triPts, scaleX, scaleY, STRETCH_STIFF, DAMP_STIFF, SHEAR_STIFF);

  // stretch, shear, bend, gravity, drag.
	trienergy(0, trii) = Re(0, 0);
	trienergy(1, trii) = Re(1, 0);
	fenergy(0, 0) += Re(0, 0);
	fenergy(1, 0) += Re(1, 0);
}

void Simulation::CuCv(int *triPts) {
	
	Matrix<double, 2, 1> Rc;
Rc = Ctri(cloth, triPts, scaleX, scaleY, STRETCH_STIFF, DAMP_STIFF, SHEAR_STIFF);

	CU(0, trii) = Rc(0, 0);
    CV(0, trii)=Rc(1, 0);
}



void Simulation::fBend(int offset) {
	int xOff = offset % cloth.xRes;
	int yOff = offset / cloth.xRes;

	// diagonal triangle pair
	int diagPts[4] = {
		offset,
		offset + 1,
		offset + cloth.xRes,
		offset + cloth.xRes + 1
	};

	// right-side triangle pair
	int rightPts[4] = {
		offset + cloth.xRes,
		offset + 1,
		offset + cloth.xRes + 1,
		offset + 2
	};

	// top-side triangle pair
	int topPts[4] = {
		offset + 1,
		offset + cloth.xRes + 1,
		offset + cloth.xRes,
		offset + 2 * cloth.xRes
	};

	bendHelper(diagPts);
	dfbendHelper(diagPts);
	triii += 1;
	EfbendHelper(diagPts);

	//if (xOff < cloth.xRes - 2){
	//	bendHelper(rightPts);
	//    dfbendHelper(rightPts);
	//	triii += 1;
	//	EfbendHelper(rightPts);
	//}
	//if (yOff < cloth.yRes - 2){
	//	bendHelper(topPts);
	//    dfbendHelper(topPts);
	//	triii += 1;
	//	EfbendHelper(topPts);
	//}
}

void Simulation::bendHelper(int *triPts) {
	
	Matrix<Vector3d, 1, 4>	R = ForceBend(cloth, triPts, DAMP_STIFF, BEND_STIFF);

	//F Bend
	for (int g = 0; g < 3; g++) {
		forces( triPts[0] * 3 + g , 0) += R(0, 0)[g];
		forces( triPts[1] * 3 + g , 0) += R(0, 1)[g];
		forces( triPts[2] * 3 + g , 0) += R(0, 2)[g];
		forces( triPts[3] * 3 + g , 0) += R(0, 3)[g];
	}
}

void Simulation::dfbendHelper(int *triPts) {

	Matrix<Matrix3d, 4, 4>	Rx = dFbendx(cloth, triPts,  DAMP_STIFF, BEND_STIFF);
	Matrix<Matrix3d, 4, 4> Rv = dFbendv(cloth, triPts,  DAMP_STIFF, BEND_STIFF);

	for (int m = 0; m < 4; ++m) for (int n = 0; n < 4; ++n) {
		for (int g = 0; g < 3; g++) for (int c = 0; c < 3; c++) {
			dforcesx(triPts[m] * 3 + g, triPts[n] * 3 + c) += Rx(m, n)(g,c);
			dforcesv(triPts[m] * 3 + g, triPts[n] * 3 + c) += Rv(m, n)(g, c);
		}
	}
}

void Simulation::EfbendHelper(int *triPts) {

	double Re = EFbend(cloth, triPts,  DAMP_STIFF, SHEAR_STIFF);

	// stretch, shear, bend, gravity, drag.
	trienergy(2, triii) = Re*.5;
	triii += 1;
	trienergy(2, triii) = Re * .5;
	fenergy(2, 0) += Re;
	
}





double *Simulation::genTrisFromMesh() {
	double *tris = new double[9 * getNumTris()];

	// copy in triangles
	for (int i = 0; i < cloth.yRes - 1; i++) {
		for (int j = 0; j < cloth.xRes - 1; j++) {
			double *triPairStart = tris + 18 * (i*(cloth.xRes - 1) + j);
			copyPoint(triPairStart, cloth.getWorldPoint(j, i));
			copyPoint(triPairStart + 3, cloth.getWorldPoint(j + 1, i));
			copyPoint(triPairStart + 6, cloth.getWorldPoint(j, i + 1));

			copyPoint(triPairStart + 9, cloth.getWorldPoint(j, i + 1));
			copyPoint(triPairStart + 12, cloth.getWorldPoint(j + 1, i));
			copyPoint(triPairStart + 15, cloth.getWorldPoint(j + 1, i + 1));
		}
	}

	return tris;
}

double *Simulation::genNorms() {
	// zero new norms
	double *norms = new double[3 * getNumPoints()];
	for (int i = 0; i < 3 * getNumPoints(); i++) norms[i] = 0;

	// additively generate norms per triangle
	for (int y = 0; y < cloth.yRes - 1; y++) {
		for (int x = 0; x < cloth.xRes - 1; x++) {
			auto p1 = Vector3d(cloth.getWorldPoint(x, y));
			auto p2 = Vector3d(cloth.getWorldPoint(x + 1, y));
			auto p3 = Vector3d(cloth.getWorldPoint(x, y + 1));
			auto norm = (p2 - p1).cross(p3 - p1);
			norm.normalize();
			for (int i = 0; i < 3; i++) {
				norms[(y     * cloth.xRes + x) * 3 + i] += norm[i];
				norms[(y     * cloth.xRes + (x + 1)) * 3 + i] += norm[i];
				norms[((y + 1) * cloth.xRes + x) * 3 + i] += norm[i];
			}

			p1 = Vector3d(cloth.getWorldPoint(x, y + 1));
			p2 = Vector3d(cloth.getWorldPoint(x + 1, y));
			p3 = Vector3d(cloth.getWorldPoint(x + 1, y + 1));
			norm = (p2 - p1).cross(p3 - p1);
			norm.normalize();
			for (int i = 0; i < 3; i++) {
				norms[((y + 1) * cloth.xRes + x) * 3 + i] += norm[i];
				norms[(y     * cloth.xRes + (x + 1)) * 3 + i] += norm[i];
				norms[((y + 1) * cloth.xRes + (x + 1)) * 3 + i] += norm[i];
			}
		}
	}

	// normalize each point's norm (for smoother jazz)
	for (int y = 0; y < cloth.yRes; y++) {
		for (int x = 0; x < cloth.xRes; x++) {
			auto norm = norms + 3 * (y * cloth.xRes + x);
			auto normv = Vector3d(norm);
			normv.normalize();
			for (int i = 0; i < 3; i++)
				norm[i] = normv[i];
		}
	}

	return norms;
}

double *Simulation::genTriNorms() {
	auto norms = genNorms();

	double *triNorms = new double[9 * getNumTris()];
	for (int y = 0; y < cloth.yRes - 1; y++) {
		for (int x = 0; x < cloth.xRes - 1; x++) {
			auto start = triNorms + 18 * (y * (cloth.xRes - 1) + x);
			copyPoint(start, norms + (y     * cloth.xRes + x) * 3);
			copyPoint(start + 3, norms + (y     * cloth.xRes + (x + 1)) * 3);
			copyPoint(start + 6, norms + ((y + 1) * cloth.xRes + x) * 3);

			copyPoint(start + 9, norms + ((y + 1) * cloth.xRes + x) * 3);
			copyPoint(start + 12, norms + (y     * cloth.xRes + (x + 1)) * 3);
			copyPoint(start + 15, norms + ((y + 1) * cloth.xRes + (x + 1)) * 3);
		}
	}

	delete norms;
	return triNorms;
}

void Simulation::copyPoint(double *dest, double *src) {
	memcpy(dest, src, 3 * sizeof(double));
}
