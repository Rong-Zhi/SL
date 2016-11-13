// **********************************
// Matlab Interface using shared libs
// **********************************
// Rueckert E., IAS, TU Darmstadt 2014
// rueckert@ias.tu-darmstadt.de
// **********************************

#include "math.h"
#include "mex.h"

#include "SL.h"
#include "FwdDynRobot.h"

//Interfaces
//[qDot] = f(q, u)
//[qDot, qDotq(derivative w.r.t. q), qDotu(derivative w.r.t. u)] = f(q, u)

int servo_enabled = false;
double   servo_time = 0.0;


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	real_robot_flag = 0;
	setRealRobotOptions();

	// VALIDATEINPUT
	if(nrhs < 2) {
		mexErrMsgTxt("call this function with two arguments: x, u");
		return;
	}
	int szX = mxGetNumberOfElements(prhs[0]); //2
	int szU = mxGetNumberOfElements(prhs[1]); //1
	const int n_dofs = szU;
	if( (szX-2*szU)*(szX-2*szU) > 1e-3 ) {
		mexErrMsgTxt("wrong input dimension, size(x,1) == 2*size(u,1)");
		return;
	}
	if(nlhs < 1 || nlhs > 3 || nlhs == 2) {
		mexErrMsgTxt("wrong number of output arguments, supported are 1 or 3 arg.");
		return;
	}

	double* x = mxGetPr(prhs[0]); //14x1
	double* u = mxGetPr(prhs[1]); //7x1

	double *outArrayPtrXdot = 0;
	double *outArrayPtrXdotX = 0;
	double *outArrayPtrXdotU = 0;
	//plhs[0] = mxCreateDoubleMatrix(szX, 1, mxREAL);
	//Faster, matrix not initialized with zeros
	plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
	mxSetM(plhs[0], szX); mxSetN(plhs[0], 1);
	mxSetData(plhs[0], mxMalloc(sizeof(double)*szX*1));
	outArrayPtrXdot = mxGetPr(plhs[0]);

	if(nlhs > 1) {
		//plhs[1] = mxCreateDoubleMatrix(szX, szX, mxREAL);
		plhs[1] = mxCreateDoubleMatrix(0, 0, mxREAL);
		mxSetM(plhs[1], szX); mxSetN(plhs[1], szX);
		mxSetData(plhs[1], mxMalloc(sizeof(double)*szX*szX));
		outArrayPtrXdotX = mxGetPr(plhs[1]);
		//plhs[2] = mxCreateDoubleMatrix(szX, szU, mxREAL);
		plhs[2] = mxCreateDoubleMatrix(0, 0, mxREAL);
		mxSetM(plhs[2], szX); mxSetN(plhs[2], szU);
		mxSetData(plhs[2], mxMalloc(sizeof(double)*szX*szU));
		outArrayPtrXdotU = mxGetPr(plhs[2]);
	}

	FwdDynRobot<NDObjectRobot> numCalculator(x, u, szX, szU,
			outArrayPtrXdot, outArrayPtrXdotX, outArrayPtrXdotU);
	numCalculator.SetModeCentralDerivative();
	//numCalculator.SetModeFwdDerivative();
	try {
		numCalculator.initiateComputation();
	}
	catch(char const* ex) { mexErrMsgTxt(ex); }
	catch(...) { mexErrMsgTxt("MexLib: computations failed"); }
}
