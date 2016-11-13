// **********************************
// Matlab Interface using shared libs
// **********************************
// Rueckert E., IAS, TU Darmstadt 2014
// rueckert@ias.tu-darmstadt.de
// **********************************

#include "math.h"
#include "mex.h"

#include "string.h"
#include "SL.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "SL_user.h"
#include "utility.h"
#include "utility_macros.h"
#include "SL_kinematics.h"
#include "QuatToEulerAngle.h"

//Interfaces
//[numEndEffector, EF, Jaccobian] = f(jointAngles)

int servo_enabled = false;
double   servo_time = 0.0;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	real_robot_flag = 0;
	setRealRobotOptions();


	int outVarId;

	// VALIDATE INPUT
	if(nrhs < 1) {
		mexErrMsgTxt("call this function with one argument i.e., the robot joint angles");
		return;
	}

	// VALIDATE OUTPUT
	if(nlhs != 3) {
		mexErrMsgTxt("only three output arguments, the number of endeffectors, the endeffector, and the Jacobian matrix, are supported");
		return;
	}

	//Input
	double* x = mxGetPr(prhs[0]);

	//Output
	//Number of endeffectors
	double *outNumEFptr = 0;
	plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
	outNumEFptr = mxGetPr(plhs[0]);
	outNumEFptr[0] = (double)N_ENDEFFS;

	//endeffector
	double *outEFptr = 0;
	plhs[1] = mxCreateDoubleMatrix(6*N_ENDEFFS, 1, mxREAL);
	outEFptr = mxGetPr(plhs[1]);

	//EFJacobians
	double *outArrayJacobian = 0; //6x5
	plhs[2] = mxCreateDoubleMatrix(0, 0, mxREAL);
	mxSetM(plhs[2], 6*N_ENDEFFS); mxSetN(plhs[2], N_DOFS);
	mxSetData(plhs[2], mxMalloc(sizeof(double)*6*N_ENDEFFS*N_DOFS));
	outArrayJacobian = mxGetPr(plhs[2]);

	try {

		if (!init_dynamics())
			return;

		int i;
		int slStartIndex = 1;
		Matrix fullJ=my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
		
		MY_MATRIX(local_joint_cog_mpos_des,0,N_DOFS,1,3);
		MY_MATRIX(local_joint_axis_pos_des,0,N_DOFS,1,3);
		MY_MATRIX(local_joint_origin_pos_des,0,N_DOFS,1,3);
		MY_MATRIX(local_link_pos_des,0,N_LINKS,1,3);
		MY_MATRIX_ARRAY(local_Alink_des,1,4,1,4,N_LINKS);
		MY_MATRIX_ARRAY(local_Ahmatdof,1,4,1,4,N_DOFS);

		for (i=0; i<=N_LINKS; ++i)
			local_Alink_des[i] = my_matrix(1,4,1,4);
		SL_Jstate state[N_DOFS+1];

		// set the current state
		for(i=0; i < N_DOFS; i++) {
			state[i+slStartIndex].th = x[i];
		}

		linkInformation(state,&base_state,&base_orient,endeff,
				local_joint_cog_mpos_des,
				local_joint_axis_pos_des,
				local_joint_origin_pos_des,
				local_link_pos_des,
				local_Alink_des,
				local_Ahmatdof);

		jacobian(local_link_pos_des,local_joint_origin_pos_des,local_joint_axis_pos_des,fullJ);

		int rowId, colId, mxArrayId;
		for(colId=0; colId < N_DOFS; colId++) {
			for(rowId=0; rowId < 6*N_ENDEFFS; rowId++) {
				mxArrayId = colId*(6*N_ENDEFFS)+rowId;
				outArrayJacobian[mxArrayId] = fullJ[rowId+slStartIndex][colId+slStartIndex];
			}
		}

		//end effector
		int endEffId, linkId;
		SL_quat curQuat;
		double orientations[4];

		for(endEffId=0; endEffId < N_ENDEFFS; endEffId++) {
			linkId = link2endeffmap[endEffId + slStartIndex]-1;

			linkQuat(local_Alink_des[linkId+slStartIndex],&curQuat);
			//my imple
			quatToEulerAngles(&curQuat, orientations);

			for(int posId = 0; posId < 3; posId++) {

				outEFptr[ endEffId*6 + posId ] =
						local_link_pos_des[linkId+slStartIndex][posId+slStartIndex];

				outEFptr[ endEffId*6 + 3 + posId ] =
						orientations[posId+slStartIndex];
			}
		}

	}
	catch(char const* ex) { mexErrMsgTxt(ex); }
	catch(...) { mexErrMsgTxt("MexLib: computations failed"); }
}
