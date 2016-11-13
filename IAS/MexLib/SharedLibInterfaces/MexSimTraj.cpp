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
//[q u] = f(l, L, q0, dt)
//[q u] = f(l, L, q0, dt, flgUseCtlLimitFBC)
//[q u] = f(l, L, q0, dt, flgUseCtlLimitFBC, stdAdditiveControlNoise)
//[q u] = f(l, L, q0, dt, flgUseCtlLimitFBC, stdAdditiveControlNoise, uMinMax)

int servo_enabled = false;
double   servo_time = 0.0;


void computeControls(SL_Jstate *jts, double *lPtr, double *LPtr, double *x, int t, int szX, int szU) {
	//jts.u = l + L*x;
	double lTimesX;
	int uId, xId;
	int slStartIndex = 1;

	for(uId = 0; uId < szU; uId++) {
		lTimesX = 0.0;
		for(xId = 0; xId < szX; xId++){
			lTimesX = lTimesX + LPtr[t*(szU*szX) + uId + xId*szU] * x[xId];
		}
		jts[uId+slStartIndex].u = (lPtr[t*szU + uId] + lTimesX);
	}
}

void applyControlNoise(SL_Jstate *jts, double stdAdditiveControlNoise, int szU) {
	int uId;
	int slStartIndex = 1;

	for(uId = 0; uId < szU; uId++) {
		jts[uId+slStartIndex].u = randn(jts[uId+slStartIndex].u, stdAdditiveControlNoise);
	}
}

void applyControlLimits(SL_Jstate *jts, double *uMaxPtr, int szU) {
	//control constraints
	int uId;
	int slStartIndex = 1;

	for(uId = 0; uId < szU; uId++) {
		if( jts[uId+slStartIndex].u < -uMaxPtr[uId] )
			jts[uId+slStartIndex].u = -uMaxPtr[uId];
		else if( jts[uId+slStartIndex].u > uMaxPtr[uId] )
			jts[uId+slStartIndex].u = uMaxPtr[uId];
	}
}

void applyLinearFBCJointConstraints(SL_Jstate *jts, int szU) {


	int uId;
	int slStartIndex = 1;
	double sensDist = 0.1;
	double alpha = 5.0;
	double beta = 0.2;
	double distNorm, uDecay;

	//soft constraint for joint angles
	for(uId = 0; uId < szU; uId++) {
		uDecay = 1.0;
		if(jts[uId+slStartIndex].th > (joint_range[slStartIndex + uId][MAX_THETA] - sensDist ) ){
			distNorm = ( jts[uId+slStartIndex].th -
					(joint_range[slStartIndex + uId][MAX_THETA] - sensDist) ) / sensDist;
			uDecay = (1+beta) * exp( -alpha*distNorm ) - beta;
		}
		else if( jts[uId+slStartIndex].th < (joint_range[slStartIndex + uId][MIN_THETA] + sensDist ) ){
			distNorm = ( (joint_range[slStartIndex + uId][MIN_THETA] + sensDist) -
					jts[uId+slStartIndex].th ) / sensDist;
			uDecay = (1+beta) * exp( -alpha*distNorm ) - beta;
		}
		jts[uId+slStartIndex].u = jts[uId+slStartIndex].u * uDecay;
	}
}

void applyMaxVelocityLimits(SL_Jstate *jts, double maxJointVel, int szU) {
	//max vel constraint
	int uId;
	int slStartIndex = 1;

	for(uId = 0; uId < szU; uId++){
		if(jts[uId+slStartIndex].thd > maxJointVel)
			jts[uId+slStartIndex].thd = maxJointVel;
		else if(jts[uId+slStartIndex].thd < -maxJointVel)
			jts[uId+slStartIndex].thd = -maxJointVel;
	}
}

double boundToMinMax(double val, double minMax){
	if(val > minMax)
		val = minMax;
	else if(val < -minMax)
		val = -minMax;
	return val;
}

void setCurrentState(double *xOutVector, double *xInArray, int t, int szX) {

	int xId;
	for(xId = 0; xId < szX; xId++)
		xOutVector[xId] = xInArray[t*szX + xId];
}

void setSL_JstateFromVector(SL_Jstate *jts, double *x, int szU) {

	int uId;
	int slStartIndex = 1;

	for(uId = 0; uId < szU; uId++) {
		jts[uId+slStartIndex].th = x[uId];
		jts[uId+slStartIndex].thd = x[szU + uId];
		jts[uId+slStartIndex].thdd = 0.0;
	}
}

void setVectorFromSL_Jstate(double *x, SL_Jstate *jts, int szU) {

	int uId;
	int slStartIndex = 1;

	for(uId = 0; uId < szU; uId++) {
		x[uId] = jts[uId+slStartIndex].th;
		x[szU + uId] = jts[uId+slStartIndex].thd;
	}
}

void storeResults(double *outArrayX, double *outArrayU, SL_Jstate *jts,  int t, int szX, int szU) {
	//store results
	int uId;
	int slStartIndex = 1;

	for(uId = 0; uId < szU; uId++) {
		outArrayX[(t+1)*szX + uId] = jts[uId+slStartIndex].th;
		outArrayX[(t+1)*szX + szU + uId] = jts[uId+slStartIndex].thd;
		outArrayU[t*szU + uId] = jts[uId+slStartIndex].u;
	}
}

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	real_robot_flag = 0;
	setRealRobotOptions();

	// VALIDATEINPUT
	if(nrhs < 4) {
		mexErrMsgTxt("call this function with at least four arguments (l, L, q0, dt)");
		return;
	}

	int szUTimesNumTimeSteps = mxGetNumberOfElements(prhs[0]); //l=[N_DOFS,T]
	int szU = N_DOFS;
	int szX = 2*N_DOFS;
	int numTimeSteps = (int)floor((double)szUTimesNumTimeSteps / (double)szU);

	//mandatory arguments
	double* lPtr = mxGetPr(prhs[0]); 		//l
	double* LPtr = mxGetPr(prhs[1]); 		//L
	double* q0Ptr = mxGetPr(prhs[2]); 		//q0
	double* dtPtr = mxGetPr(prhs[3]);		//dt
	double slTimeStep = dtPtr[0];

	//optional arguments
	double* uMaxPtr = 0;
	double* flgUseCtlLimitFBCPtr = 0;
	double* stdAdditiveControlNoisePtr = 0;
	double stdAdditiveControlNoise = 0.0;
	bool flgUseCtlLimitFBC = false;
	bool flgApplyControlLimits = false;
	bool flgApplyAdditiveNoise = false;

	if(nrhs >= 5) {
		flgUseCtlLimitFBCPtr = mxGetPr(prhs[4]);
		if(flgUseCtlLimitFBCPtr[0] > 0.0) {
			read_sensor_offsets(config_files[SENSOROFFSETS]);
			flgUseCtlLimitFBC = true;
		}
	}
	if(nrhs >= 6) {
		flgApplyAdditiveNoise = true;
		stdAdditiveControlNoisePtr = mxGetPr(prhs[5]);
		stdAdditiveControlNoise = stdAdditiveControlNoisePtr[0];
	}
	if(nrhs >= 7) {
		uMaxPtr = mxGetPr(prhs[6]);
		flgApplyControlLimits = true;
	}

	//allocate output memory
	double *outArrayX = 0; //10xn
	double *outArrayU = 0;	//5xn

	plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
	mxSetM(plhs[0], szX); mxSetN(plhs[0], numTimeSteps);
	mxSetData(plhs[0], mxMalloc(sizeof(double)*szX*numTimeSteps));
	outArrayX = mxGetPr(plhs[0]);

	plhs[1] = mxCreateDoubleMatrix(0, 0, mxREAL);
	mxSetM(plhs[1], szU); mxSetN(plhs[1], numTimeSteps-1);
	mxSetData(plhs[1], mxMalloc(sizeof(double)*szU*(numTimeSteps-1)));
	outArrayU = mxGetPr(plhs[1]);

	int slStartIndex = 1;
	int t, tSim, xId, uId;
	double x[szX];
	SL_Jstate jts[N_DOFS+1];
	SL_uext   ux[N_DOFS+1];
	bzero((void *)jts,sizeof(SL_Jstate)* (N_DOFS+1) );
	bzero((void *)ux,sizeof(SL_uext)* (N_DOFS+1) );

	double maxStepVel = 0.15;
	double maxStepAcc = 2.0;
	double maxJointVel = 100.0;
	double stepVel, stepAcc;

	try {
		if (!init_dynamics())
			return;

		setCurrentState(outArrayX, q0Ptr, 0, szX); //set output array to initial state
		for(uId = 0; uId < szU; uId++){
			jts[uId+slStartIndex].th = q0Ptr[uId];
			jts[uId+slStartIndex].thd = q0Ptr[uId + szU];
		}

		for(t=0; t < numTimeSteps-1; t++)
		{
			setCurrentState(x, outArrayX, t, szX);//q_t
			setSL_JstateFromVector(jts, x, szU);//q_t+1
			computeControls(jts, lPtr, LPtr, x, t, szX, szU);//u_t

			if(flgApplyAdditiveNoise) {
				applyControlNoise(jts, stdAdditiveControlNoise, szU);
			}

			if(flgApplyControlLimits) {
				applyControlLimits(jts, uMaxPtr, szU);
			}
			if(flgUseCtlLimitFBC) {
				applyLinearFBCJointConstraints(jts, szU);
			}

			SL_ForDyn(jts, &base_state, &base_orient, ux, endeff);

			for(uId = 0; uId < szU; uId++){

				stepAcc = slTimeStep*jts[uId+slStartIndex].thdd;
				stepAcc = boundToMinMax(stepAcc, maxStepAcc);
				jts[uId+slStartIndex].thd = jts[uId+slStartIndex].thd + stepAcc;

				stepVel = slTimeStep*jts[uId+slStartIndex].thd;
				stepVel = boundToMinMax(stepVel, maxStepVel);
				jts[uId+slStartIndex].th = jts[uId+slStartIndex].th + stepVel;

			}

			applyMaxVelocityLimits(jts, maxJointVel, szU);
			storeResults(outArrayX, outArrayU, jts,  t, szX, szU);

		}

	}
	catch(char const* ex) { mexErrMsgTxt(ex); }
	catch(...) { mexErrMsgTxt("MexLib: computations failed"); }
}
