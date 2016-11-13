// **********************************
// Matlab Interface using shared libs
// **********************************
// Rueckert E., IAS, TU Darmstadt 2014
// rueckert@ias.tu-darmstadt.de
// **********************************

#include "math.h"
#include "mex.h"
#include "utility.h"
#include "SL_system_headers.h"
#include "SL.h"
#include "SL_dynamics.h"
#include "SL_user.h"
#include "SL_common.h"
#include "Randn.h"

//Interfaces
//[qIntegrated] = f(q, u, dt)

int servo_enabled = false;
double   servo_time = 0.0;


void storeResults(double *outArrayX, SL_Jstate *jts,  int t, int szX, int szU) {
	//store results
	int uId;
	int slStartIndex = 1;

	for(uId = 0; uId < szU; uId++) {
		outArrayX[(t)*szX + uId] = jts[uId+slStartIndex].th;
		outArrayX[(t)*szX + szU + uId] = jts[uId+slStartIndex].thd;
	}
}

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//Interfaces
//[qnext] = f(q, u, dt)
//q ... DoF*2 times n time steps
//u ... DoF times n time steps
//qnext ... same dim as q
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	real_robot_flag = 0;
	setRealRobotOptions();


	// VALIDATEINPUT
	if(nrhs < 3) {
		mexErrMsgTxt("call this function with at least three arguments (q, u, dt)");
		return;
	}

	int szQTimesNumTimeSteps = mxGetNumberOfElements(prhs[0]); //x=[N_DOFS*2,T]
	int szU = N_DOFS;
	int szQ = 2*N_DOFS;
	int numTimeSteps = (int)floor((double)szQTimesNumTimeSteps / (double)szQ);

	//mandatory arguments
	double* qPtr = mxGetPr(prhs[0]); 		//q
	double* uPtr = mxGetPr(prhs[1]); 		//u
	double* dtPtr = mxGetPr(prhs[2]);		//dt
	double slTimeStep = dtPtr[0];

	//allocate output memory
	double *outArrayX = 0; //10xn
	plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
	mxSetM(plhs[0], szQ); mxSetN(plhs[0], numTimeSteps);
	mxSetData(plhs[0], mxMalloc(sizeof(double)*szQ*numTimeSteps));
	outArrayX = mxGetPr(plhs[0]);

	int slStartIndex = 1;
	int t, tSim, xId, uId;
	double stepVel, stepAcc;

	SL_Jstate jts[N_DOFS+1];
	SL_uext   ux[N_DOFS+1];
	bzero((void *)jts,sizeof(SL_Jstate)* (N_DOFS+1) );
	bzero((void *)ux,sizeof(SL_uext)* (N_DOFS+1) );

	try {
		if (!init_dynamics())
			return;

		for(t=0; t < numTimeSteps; t++)
		{
			for(uId = 0; uId < szU; uId++){
				jts[uId+slStartIndex].th 	= qPtr[t*szQ + uId];
				jts[uId+slStartIndex].thd 	= qPtr[t*szQ + uId + szU];
				jts[uId+slStartIndex].u 	= uPtr[t*szU + uId];
				jts[uId+slStartIndex].thdd = 0.0;
			}

			SL_ForDyn(jts, &base_state, &base_orient, ux, endeff);

			for(uId = 0; uId < szU; uId++){

				stepAcc = slTimeStep*jts[uId+slStartIndex].thdd;
				//stepAcc = boundToMinMax(stepAcc, maxStepAcc);
				jts[uId+slStartIndex].thd = jts[uId+slStartIndex].thd + stepAcc;

				stepVel = slTimeStep*jts[uId+slStartIndex].thd;
				//stepVel = boundToMinMax(stepVel, maxStepVel);
				jts[uId+slStartIndex].th = jts[uId+slStartIndex].th + stepVel;

			}

			//applyMaxVelocityLimits(jts, maxJointVel, szU);
			storeResults(outArrayX, jts,  t, szQ, szU);

		}

	}
	catch(char const* ex) { mexErrMsgTxt(ex); }
	catch(...) { mexErrMsgTxt("MexLib: computations failed"); }
}
