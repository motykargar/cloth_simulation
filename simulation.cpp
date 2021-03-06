#include "simulation.h"

using namespace std;
Simulation::Simulation(int clothXRes, int clothYRes)
	: cloth(Cloth(clothXRes, clothYRes)) {
	cloth = Cloth(cloth.xRes, cloth.yRes);

	frameRate =  25;
	frame = 0;
	size = 0;
	////std::cout << getNumTris() << std::endl;
	scaleX = 1;
	scaleY = 1;
	forcesave.resize(3 * getNumPoints(), 1);
	forces.resize(3 * getNumPoints(), 1);
	dforcesx.resize(3 * getNumPoints(), 3 * getNumPoints());
	dforcesv.resize(3 * getNumPoints(), 3 * getNumPoints());
	M.resize(3 * getNumPoints(), 3 * getNumPoints());
	S.resize(3 * getNumPoints(), 3 );
	ri.resize(3 * getNumPoints(), 1);
	z0.resize(3 * getNumPoints(), 1);

	x.resize(3 * getNumPoints(), 1);
	A.resize(3 * getNumPoints(), 3 * getNumPoints());
	b.resize(3 * getNumPoints(), 1);
	
	V.resize(3 * getNumPoints(), 1);
	y.resize(3 * getNumPoints(), 1);
	P.resize(3 * getNumPoints(), 3 * getNumPoints());
	Pinv.resize(3 * getNumPoints(), 3 * getNumPoints());
	//Matrix<double, 5, 1> fenergy;  // stretch, shear, bend, gravity, drag.
	trienergy.resize(5, getNumTris());
	
	fenergy.setZero();
	trienergy.setZero();
	
	forcesave.setZero();
   forces.setZero();
    dforcesx.setZero();
    dforcesv.setZero();
    M.setZero();
    S.setZero();
    z0.setZero();
    A.setZero();
    b.setZero();
	
	
   // regenerate triangles from the mesh
   triVerts = genTrisFromMesh();
   norms = genTriNorms();
  doFinaleInPre = true;
  inStep = false;
  stepSuccessFlag = true;
  nbInternalStepsInc = NB_INTERNAL_STEPS_INC_DEFAULT;
  nbInternalSteps = 0;
  

	for (int i = 0; i < cloth.yRes - 1; i++) {
		for (int j = 0; j < cloth.xRes - 1; j++) {
			int offset = i * cloth.xRes + j;
		//	std::cout << "offset" << offset << std::endl;
			for (int m = 0; m < 3; m++) {
				M((offset)*3 + m, (offset) * 3 + m) += DENSITY * cloth.triUvArea / 3;
				M((offset+1) * 3 + m, (offset+1) * 3 + m) += DENSITY * cloth.triUvArea / 3;
				M((offset + cloth.xRes) * 3  + m, (offset + cloth.xRes) * 3  + m) += DENSITY * cloth.triUvArea / 3;
				M((offset + cloth.xRes) * 3 + m, (offset + cloth.xRes) * 3 + m) += DENSITY * cloth.triUvArea / 3;
				M((offset + 1) * 3 + m, (offset + 1) * 3 + m) += DENSITY * cloth.triUvArea / 3;
				M((offset + cloth.xRes+1) * 3 + m, (offset + cloth.xRes+1) * 3 + m) += DENSITY * cloth.triUvArea / 3;
			}
		}
	}

	for (int pt = 0; pt < (cloth.xRes * cloth.yRes)-2; pt++) {
		for (int m = 0; m < 3; m++) {	
			S((pt * 3) + m, m) = 1;
		}
	}
	
///	/////removeAllConstraints

	maxSubSteps = 1000; //
	h = 0.02;// TIMESTEP;
	
	TIME = 0;
	setFrameRate();
	changeStep(0);
	preSubSteps();
	
}

void Simulation::update() {
	// if simulation is paused, don't update

	
	
	float stepEnd = (frame + 1) / frameRate;

	
	nbSubSteps = 0;
	doneSubSteps = false;
	fullStep = true;
	inStep = true;
	stepFailed = false;

	if (doneSubSteps) running = false;
	if (!running) return;

		if (inStep){
		
			while ((!done)&(nbSubSteps < maxSubSteps))
			{
			
				++nbSubSteps;
				stepCG();

			}

			postSubSteps();
			if (stepSuccessFlag) {
				if (fullStep) {
					++ nbInternalSteps;
					if (nbInternalSteps == nbInternalStepsInc) {
						nbInternalSteps = 0;
						changeStep(1);
					}
					if (TIME >= stepEnd) {	
						doneSubSteps = true;
					}
					else {
						float fudge = 0.001f;
						float diff = TIME + h - stepEnd;
						if (diff > fudge) {
							fullStep = false;
								h = stepEnd - TIME;	
						}
						else if (diff > -fudge) {
							h = stepEnd - TIME;
						}
						else {
							h = h;
						}
						preSubSteps();
						nbSubSteps = 0;
					}
				}
				else {
					doneSubSteps = true;
				}
			}
			else {
				stepFailed = true;
			}
		if (stepFailed) {
			stepFailed = false;

			if (nbInternalSteps == 0) {
				
				changeStep(-1);
				if ((nbInternalStepsInc * INC_CHANGE_FACTOR)>MAX_NB_INTERNAL_STEPS_INC)
				{
					nbInternalStepsInc = MAX_NB_INTERNAL_STEPS_INC;
			}
				else {
				nbInternalStepsInc = (nbInternalStepsInc * INC_CHANGE_FACTOR);
				}
			
			}
			else {
				
			changeStep(-1);
				nbInternalStepsInc = NB_INTERNAL_STEPS_INC_DEFAULT;
			nbInternalSteps = 0;
		}
	
			preSubSteps();
			nbSubSteps = 0;
		}
	}
		
		
		if (triVerts) delete triVerts;
		if (norms) delete norms;
		triVerts = genTrisFromMesh();
		norms = genTriNorms();
	
	inStep = false;
	++ frame;
	/////
	
	
	std::ostringstream convert;
	convert << frame;
	fstream output_file(convert.str().c_str(), ios::out);
	int a = 0, b = 0;

		for (b = 0; b<cloth.xRes * cloth.yRes*3; b++)
		{
			
			output_file <<  cloth.worldPoints[b]  << endl;
		}
		
	/////
}

void Simulation::update2() {
	// if simulation is paused, don't update




	if (frame>195) running = false;
	if (!running) return;
	std::ostringstream convert;
	convert << frame;
	ifstream input_file(convert.str().c_str(), ios::in);
	int a = 0, b = 0;

	for (b = 0; b<cloth.xRes * cloth.yRes * 3; b++)
	{

		input_file >> cloth.worldPoints[b] ;
	}
	
	if (triVerts) delete triVerts;
	if (norms) delete norms;
	triVerts = genTrisFromMesh();
	norms = genTriNorms();

	
	++frame;
	/////


	

	/////
}



void Simulation::preSubSteps() {
	//dforcesx.setZero();
	//dforcesv.setZero();
	if (doFinaleInPre) {
		
		postSubStepsFinale();
		doFinaleInPre = false;
	
	}
	
	inStep = true;
	
	for (int i = 0; i < cloth.yRes - 1; i++) {
		for (int j = 0; j < cloth.xRes - 1; j++) {
			int offset = i * cloth.xRes + j;
	fBend(offset);
		}
	}

	
	for (int pt = 0; pt < (cloth.xRes * cloth.yRes); pt++) {	
		forces(3*pt + 1) -= M((pt * 3) + 1, (pt * 3) + 1)*9.8;
		
	}
	


	for (int i = 0; i < cloth.xRes * cloth.yRes; i++) {
		auto	v = Vector3d(cloth.getWorldVel(i));
		V((i * 3) + 0, 0) = v[0];
		V((i * 3) + 1, 0) = v[1];
		V((i * 3) + 2, 0) = v[2];
	
	}



	A = dforcesx * (-h * h) + dforcesv * (-h) + M;
	b = h * (h * dforcesx * V + forces);
	for (int i = 0; i < 3* cloth.xRes * cloth.yRes; i++) {
		y(i,0)=cloth.lastDeltaV0[i];
	}
	
	z0 = z0 * h;
	preStepCG();


}
void Simulation::postSubStepsFinale() {

	
	
	forces.setZero();
	dforcesx.setZero();
	dforcesv.setZero();

	stepSuccessFlag = true;
	
	int ID = 0;
	for (int i = 0; i < cloth.yRes - 1; i++) {
		for (int j = 0; j < cloth.xRes - 1; j++) {
			int offset = i * cloth.xRes + j;
			
			fShearStretch(offset, ID);
			
			ID += 1;
		}
	}

	for (int i = 0; i < 2 * (cloth.xRes - 1) * (cloth.yRes - 1); i++) {
	
		if (abs(cloth.Cusave[i] - cloth.Cu[i]) > STRETCHLIMIT ||
			abs(cloth.Cvsave[i] - cloth.Cv[i]) > STRETCHLIMIT)
			 {
			stepSuccessFlag = false;

		}
	
			
	}

}
void Simulation::postSubSteps() {
	stepCG();

	for (int i = 0; i <  3 * cloth.xRes * cloth.yRes; i++) {
		cloth.worldVelssave[i] = cloth.worldVels[i];
		cloth.lastDeltaV0save[i] = cloth.lastDeltaV0[i];
		cloth.worldPointssave[i] = cloth.worldPoints[i];
		forcesave(i) = forces(i);
	}
	for (int i = 0; i < 2 * (cloth.xRes - 1) * (cloth.yRes - 1); i++) {
		cloth.Cusave[i] = cloth.Cu[i];
		cloth.Cvsave[i] = cloth.Cv[i];
	}
	

	for (int i = 0; i < 3 * cloth.xRes * cloth.yRes; i++) {
		//std::cout << "r" << x(i, 0) << std::endl;
		cloth.lastDeltaV0[i] = x(i, 0);
		cloth.worldVels[i] += x(i, 0);
		
	}
	for (int i = 0; i < 3 * cloth.xRes * cloth.yRes; i++) {
		cloth.worldPoints[i] += h * cloth.worldVels[i];
	//std::cout << "m" << h * cloth.worldVels[i] << std::endl;
	}

	TIME +=h;
	std::cout << "TIME2" << TIME << std::endl;
	postSubStepsFinale();
	inStep = false;
	
		 if (!stepSuccessFlag) {
			 
			 for (int i = 0; i < 3 * cloth.xRes * cloth.yRes; i++) {
				 cloth.worldVels[i] = cloth.worldVelssave[i];
				 cloth.lastDeltaV0[i] = cloth.lastDeltaV0save[i];
				 cloth.worldPoints[i] = cloth.worldPointssave[i];
				 forces(i) = forcesave(i);
			 }
			 for (int i = 0; i < 2 * (cloth.xRes - 1) * (cloth.yRes - 1); i++) {
				 cloth.Cu[i] = cloth.Cusave[i];
				 cloth.Cv[i] = cloth.Cvsave[i];
			 }
			 doFinaleInPre = true;
			 TIME -= h;
			 return;
		 }
	// }
}


void Simulation::fShearStretch(int offset, int ID) {
	int botLeftTri[3] = { offset, offset + 1, offset + cloth.xRes };
	int topRightTri[3] = { offset + cloth.xRes, offset + 1, offset + cloth.xRes+ 1 };

	fShearStretchHelper(botLeftTri,(ID*2));
	fShearStretchHelper(topRightTri, ((ID * 2)+1));
	
}

void Simulation::fShearStretchHelper(int *triPts, int id) {
	
	Cstrech cstrech(cloth, triPts, scaleX, scaleY, id);
	Cshear cshear(cloth, triPts);

	// Fstretch
	//cloth.Force[tri[0] * 3 + 0] += 1;
	for (int mm = 0; mm < 3; mm++) {
		Vector3d forcest = (-STRETCH_STIFF * (cstrech.dcu_dxm(mm, 0) *cstrech.cu
			+ cstrech.dcv_dxm(mm, 0) * cstrech.cv));
		Vector3d forcesh = (-SHEAR_STIFF * cshear.dc_dxm(mm, 0)* cshear.c);
		Vector3d dampst = (-DAMP_STIFF * (cstrech.dcu_dxm(mm, 0) *cstrech.dcu_dt
			+ cstrech.dcv_dxm(mm, 0) * cstrech.dcv_dt));
		Vector3d dampsh = (-DAMP_STIFF * cshear.dc_dxm(mm, 0) * cshear.dc_dt);
		for (int nn = 0; nn < 3; nn++) {

			forces((3 * triPts[mm]) + nn) += forcest[nn];
			forces((3 * triPts[mm]) + nn) += dampst[nn];
			forces((3 * triPts[mm]) + nn) += forcesh[nn];
			forces((3 * triPts[mm]) + nn) += dampsh[nn];
		}
	}

	for (int m = 0; m < 3; ++m) for (int n = 0; n < 3; ++n) {

		// dFstetch
		Matrix3d df= -DAMP_STIFF * (
			(cstrech.dcu_dxm(m, 0)* cstrech.dcu_dxm(n, 0).transpose())
			+ (cstrech.dcv_dxm(m, 0)* cstrech.dcv_dxm(n, 0).transpose()));
		df+= -DAMP_STIFF *
			(cshear.dc_dxm(m, 0)* cshear.dc_dxm(n, 0).transpose());

		for (int mm = 0; mm < 3; ++mm) for (int nn = 0; nn < 3; ++nn) {

			dforcesv(3 * triPts[n] + nn, 3 * triPts[m] + mm) += df(nn, mm);
		}


		df.setZero();
		 df += -STRETCH_STIFF * (
			(cstrech.dcu_dxm(m, 0)* (cstrech.dcu_dxm(n, 0).transpose()))
			+ (cstrech.dcv_dxm(m, 0)* cstrech.dcv_dxm(n, 0).transpose())
			+ cstrech.d2cu_dxmdxn(m, n) * cstrech.cu
			+ cstrech.d2cv_dxmdxn(m, n) * cstrech.cv);
		df += -DAMP_STIFF * (
			cstrech.d2cu_dxmdxn(m, n) * cstrech.dcu_dt
			+ cstrech.d2cv_dxmdxn(m, n) * cstrech.dcv_dt);
		// dFshear
		df += -SHEAR_STIFF * (
			(cshear.dc_dxm(m, 0)* cshear.dc_dxm(n, 0).transpose())
			+ cshear.d2c_dxmdxn(m, n) * cshear.c*(MatrixXd::Identity(3, 3)));
		df += (MatrixXd::Identity(3, 3)) * (-DAMP_STIFF * (
			cshear.d2c_dxmdxn(m, n) * cshear.dc_dt));


		for (int mm = 0; mm < 3; ++mm) for (int nn = 0; nn < 3; ++nn) {

			dforcesx(3 * triPts[n] + nn, 3 * triPts[m] + mm) += df(nn, mm);
		}

	}

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
	

	if (xOff < cloth.xRes - 2)
	{
		bendHelper(rightPts);
		
	}
		

	if (yOff < cloth.yRes - 2)
	{
		bendHelper(topPts);
		
	}
		


	
	
	
}

void Simulation::bendHelper(int *triPts) {

		Bend bend(cloth, triPts);

		for (int m = 0; m < 4; m++) {
			Vector3d forcebend = -BEND_STIFF * bend.dc_dxm(m, 0) * bend.c;
			Vector3d dampbend = -DAMP_STIFF * bend.dc_dxm(m, 0) * bend.dc_dt;

			for (int nn = 0; nn < 3; nn++) {

				forces((triPts[m] * 3) + nn) += forcebend[nn];
				forces((triPts[m] * 3) + nn) += dampbend[nn];

			}

		}
		for (int m = 0; m < 4; ++m) for (int n = 0; n < 4; ++n) {
			
			Matrix3d df = -DAMP_STIFF * (bend.dc_dxm(m, 0)* bend.dc_dxm(n, 0).transpose());

			for (int mm = 0; mm < 3; ++mm) for (int nn = 0; nn < 3; ++nn) {

				dforcesv(3 * triPts[n] + nn, 3 * triPts[m] + mm) += df(nn, mm);
			}
			df.setZero();
			df += -BEND_STIFF * ((bend.dc_dxm(m, 0)* bend.dc_dxm(n, 0).transpose())
				+ bend.d2c_dxmdxn(m, n) * bend.c);
			df += -DAMP_STIFF * bend.d2c_dxmdxn(m, n) * bend.dc_dt;

			for (int mm = 0; mm < 3; ++mm) for (int nn = 0; nn < 3; ++nn) {

				dforcesx(3 * triPts[n] + nn, 3 * triPts[m] + mm) += df(nn, mm);
			}
			
		}



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

void Simulation::preStepCG() {

	P.setZero();
	Pinv.setZero();
	for (int i = 0; i < 3*getNumPoints(); ++i)
			if (!(A(i , i ) == 0)) {
				P.insert(i , i ) = A(i , i );
				Pinv.insert(i , i ) = 1 / A(i , i );
		}
	x = z0;
	filterCompInPlace(x); /////
	bhat = -(A * x);
	bhat += b;
	filterInPlace(bhat); /////
	delta0 = (P * bhat).dot(bhat);
	x += filter(y); ////
	r = x;
	c = x;
	r = filter(b - A * x);  ////
	c = filter(Pinv * r);   ////
	deltaNew = r.dot(c);

	double nbSteps = 0;
	done = false;

}

void Simulation::filterCompInPlace(VectorXd b) {
	Matrix3d s;
	Vector3d a;
	Vector3d r;
	s.setZero();
	for (int pt = 0; pt < cloth.xRes * cloth.yRes; pt++) {
		//s.setZero();
		for (int m = 0; m < 3; m++) {
			
			s(m, m) = S((pt * 3) + m, m);
			a[m] = b[(pt * 3) + m];
		}
		r = (I-s) * a;
		for (int m = 0; m < 3; m++) {
			b[(pt * 3) + m] = r[m];
		}
	}


}
void Simulation::filterInPlace(VectorXd b) {
	
	Matrix3d s;
	Vector3d a;
	Vector3d r;
	s.setZero();
	for (int pt = 0; pt < cloth.xRes * cloth.yRes; pt++) {
		//s.setZero();
		for (int m = 0; m < 3; m++) {
			
			s(m, m) = S((pt * 3) + m, m);
			a[m] = b[(pt * 3) + m];
		}
		r = s * a;
		for (int m = 0; m < 3; m++) {
			b[(pt * 3) + m] = r[m];
		}
	}
	

}

ForceMatrix Simulation::filter(ForceMatrix y) {
	ri.setZero();
	Matrix3d s;
	Vector3d a;
	Vector3d r;
	s.setZero();
	for (int pt = 0; pt < cloth.xRes * cloth.yRes; pt++) {
	//	s.setZero();
		for (int m = 0; m < 3; m++) {
			s(m, m) = S((pt * 3) + m, m);
			a[m] = y((pt * 3) + m,0);	
			//std::cout << "s" << s(m, m) << std::endl;
		}
		r = s * a; 
		//std::cout << "r" << r[0] << std::endl;
		//std::cout << "r" << r[1] << std::endl;
		//std::cout << "r" << r[2] << std::endl;
		for (int m = 0; m < 3; m++) {
			ri((pt * 3) + m, 0) = r[m];
		}
	}
	return ri;
}

ForceMatrix Simulation::filterComp(ForceMatrix y) {
	ri.setZero();
	Matrix3d s;
	Vector3d a;
	Vector3d r;
	s.setZero();
	for (int pt = 0; pt < cloth.xRes * cloth.yRes; pt++) {
	//	s.setZero();
		for (int m = 0; m < 3; m++) {
			s(m, m) = S((pt * 3) + m, m);
			a[m] = y((pt * 3) + m, 0);
		}
		r = (I-s) * a;
		for (int m = 0; m < 3; m++) {
			ri((pt * 3) + m, 0) = r[m];
		}
	}
	return ri;
}

void Simulation::stepCG()
{
	VectorXd q(3 * cloth.xRes * cloth.yRes);
	//resize(3 * cloth.xRes * cloth.yRes);
	VectorXd s(3 * cloth.xRes * cloth.yRes);
	//resize(3 * cloth.xRes * cloth.yRes);
	if (deltaNew < TOLERANCE * TOLERANCE * delta0) {
		done = true;
		return;
	}
	//std::cout << "deltaNew" << TOLERANCE * TOLERANCE * delta0 << std::endl;
	q=A*c;
	filterInPlace(q);
	double alpha = deltaNew / (c.dot(q));
	
	x += alpha * c;
	r += -alpha * q;
	s=Pinv*r;
	//Pinv*r;
	double deltaOld = deltaNew;
	deltaNew = r.dot(s);
	c *= deltaNew / deltaOld;
	c += s;
	filterInPlace(c);
	++ nbSteps;
}
void Simulation::multiply(VectorXd destV, MatrixXd srcM, VectorXd srcV) 
{
	for (int row = 0; row =!3 * cloth.xRes * cloth.yRes; ++row) {
		destV[row] = 0;
	}
	for (int rit = 0; rit != 3*cloth.xRes * cloth.yRes; ++rit) {
		for (int row=0; row != 3*cloth.xRes * cloth.yRes; ++row) {
			destV[row] += srcM(rit, row)*srcV[row];
		}
	}
}

void Simulation::changeStep(int inc)
{
	static const int maxDenom = 6;
	static const int minPow = -3;
	static const int maxSize = maxDenom - 1;
	if (maxSize > size + inc) { size = size + inc; }
	else { size = maxSize; }
	if (size >= 0) {
		// In this regime, the timestep is a constant fraction of the frame
		// period. The size defines the denominator of the fraction.
		const int denom = maxSize - size + 1;
		h = (1.f / frameRate) / denom;
	}
	else {
		// In this regime, the timestep is a power-of-2 fraction of the
		// frame period. The size defines the exponent of the power of two.
		const int exp = size + 1 + minPow;
		//DGFX_ASSERT(exp < minPow && exp <= 0);
		h = (1.f / frameRate) / (1 << -exp);
	}
}

void Simulation::setFrameRate()
{
	framePeriod = (1.f / frameRate);
	if (h > framePeriod) { h = framePeriod; }
	frame = TIME / framePeriod;
	
	nbInternalStepsInc = NB_INTERNAL_STEPS_INC_DEFAULT;
	nbInternalSteps = 0;
}

