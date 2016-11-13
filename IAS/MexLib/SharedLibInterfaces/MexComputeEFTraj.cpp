// **********************************
// Matlab Interface using shared libs
// **********************************
// Rueckert E., IAS, TU Darmstadt 2014
// rueckert@ias.tu-darmstadt.de
// **********************************
// **********************************
// Lioutikov R., IAS, TU Darmstadt 2016
// lioutikov@ias.tu-darmstadt.de
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

int servo_enabled = false;
double   servo_time = 0.0;


void printUsage(){
	char usage[5*1024];
	int usageOffset,idx;

	usageOffset = sprintf(usage,"Usage: libMexComputeEFTraj%s <joints> <inEuler> <endeff>\n\t<joints> [%dxN]: joint trajectory of N timeSteps.\n\t<inEuler> [true/false]: If true the orientation is returned in euler angles (x,y,z order) otherwise in quaternions. Default is false.\n\t<endeff> [%dx6]: The rigid offsets from the last link to the respective endeffectors. The six entries correspond to x,y,z offset and x,y,z euler angles for each end effector. Defaults are:\n\t\t[",ROBOT_NAME, N_DOFS,N_ENDEFFS);

	setDefaultEndeffector();

	for (idx = 1; idx <=N_ENDEFFS; idx++){
		usageOffset += sprintf(usage+usageOffset,"%f, %f, %f, %f, %f, %f\n\t\t ",endeff[idx].x[_X_],endeff[idx].x[_Y_],endeff[idx].x[_Z_],endeff[idx].a[_A_],endeff[idx].a[_B_],endeff[idx].a[_G_]);
	}

	sprintf(usage+usageOffset-4,"]\n\n\tReturns <numEndeff>, <endeffPose>\n\t<numEndeff> [1x1]: the number of end effectors. Currently <numEndeff> is always %d.\n\t<endeffPose> [<dim>*%dxN]: The end effector poses for each given joint state. <dim> is either 6 or 7 depending if <inEuler> is true or false.",N_ENDEFFS,N_ENDEFFS);

	mexErrMsgTxt(usage);
}

double* getEndeffOffset(const mxArray* ptr){
	if (mxGetNumberOfElements(ptr) != N_ENDEFFS*6){
		mexErrMsgTxt("Wrong number of end effector offsets.");
		return 0;
	}

	return mxGetPr(ptr);
}

int getInEuler(const mxArray* ptr){
	if (! mxIsLogicalScalar(ptr)){
		mexErrMsgTxt("The Argument inEuler must be either true or false");
		return -1;
	}

	return *mxGetLogicals(ptr);
}

void quatToEulerXYZ(double *quat, double *XYZ){
	XYZ[0] = atan2(-2*(quat[2]*quat[3] - quat[0]*quat[1]), pow(quat[0],2) - pow(quat[1],2) - pow(quat[2],2) + pow(quat[3],2));
	XYZ[1] = asin(2*(quat[1]*quat[3] + quat[0]*quat[2]));
	XYZ[2] = atan2(-2*(quat[1]*quat[2] - quat[0]*quat[3]), pow(quat[0],2) + pow(quat[1],2) - pow(quat[2],2) - pow(quat[3],2));
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

	//Print usage information
	if( (nrhs < 1) || (nrhs > 3) ) {
		printUsage();
		return;
	}

	real_robot_flag = 0;
	setRealRobotOptions();
		

	//Retrieve passed arguments
	double* states = mxGetPr(prhs[0]);
	int inEuler = 0;
	double* endeffOff = 0;
	if(nrhs > 1){
		if(mxGetNumberOfElements(prhs[1]) > 1){
			if(!(endeffOff = getEndeffOffset(prhs[1]))){
				return;
			}
			if(nrhs > 2){
				printUsage();
				return;
			}
		}else{
			if((inEuler = getInEuler(prhs[1])) == -1){
				return;
			}

			if(nrhs > 2){
				if(!(endeffOff = getEndeffOffset(prhs[2]))){
					return;
				}
			}
		}
	}

	int numTimeSteps = mxGetNumberOfElements(prhs[0]) / N_DOFS;
	int dimOut = inEuler ? 6 :7;

	//Set up return variables
	
	//Number of endeffectors
	double *outNumEFptr = 0;
	plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
	outNumEFptr = mxGetPr(plhs[0]);
	outNumEFptr[0] = (double)N_ENDEFFS;

	//end effector pose for each time step
	plhs[1] = mxCreateDoubleMatrix(0, 0, mxREAL);
	mxSetM(plhs[1], dimOut*N_ENDEFFS); mxSetN(plhs[1], numTimeSteps);
	mxSetData(plhs[1], mxMalloc(sizeof(double)*dimOut*N_ENDEFFS*numTimeSteps));
	double *outEFptr = mxGetPr(plhs[1]);

	//compute endefector pose via forward kinematics
	int timeStep, linkId, idx;
	try {


		if (!init_dynamics()){
			return;
		}

		if (endeffOff){
			for(idx = 1; idx <= N_ENDEFFS; idx++){
				endeff[idx].x[_X_] = endeffOff[(idx-1)*6+0];
				endeff[idx].x[_Y_] = endeffOff[(idx-1)*6+1];
				endeff[idx].x[_Z_] = endeffOff[(idx-1)*6+2];
				endeff[idx].a[_A_] = endeffOff[(idx-1)*6+3];
				endeff[idx].a[_B_] = endeffOff[(idx-1)*6+4];
				endeff[idx].a[_G_] = endeffOff[(idx-1)*6+5];
			}
		
		}
		SL_Jstate state[N_DOFS+1];
		SL_quat curQuat;
		double eulerAngles[3];
		
		MY_MATRIX(local_joint_cog_mpos_des,0,N_DOFS,1,3);
		MY_MATRIX(local_joint_axis_pos_des,0,N_DOFS,1,3);
		MY_MATRIX(local_joint_origin_pos_des,0,N_DOFS,1,3);
		MY_MATRIX(local_link_pos_des,0,N_LINKS,1,3);
		MY_MATRIX_ARRAY(local_Alink_des,1,4,1,4,N_LINKS);
		MY_MATRIX_ARRAY(local_Ahmatdof,1,4,1,4,N_DOFS);
		for (idx=0; idx<=N_LINKS; ++idx){
			local_Alink_des[idx] = my_matrix(1,4,1,4);
		}

		
		
		for(timeStep=0; timeStep < numTimeSteps; timeStep++) {

			for(idx=1; idx <= N_DOFS; idx++) {
				state[idx].th = states[timeStep*N_DOFS+idx-1];
			}

			linkInformation(state,&base_state,&base_orient,endeff,
					local_joint_cog_mpos_des,
					local_joint_axis_pos_des,
					local_joint_origin_pos_des,
					local_link_pos_des,
					local_Alink_des,
					local_Ahmatdof);

			for(idx=1; idx <= N_ENDEFFS; idx++) {
				linkId = link2endeffmap[idx];
				linkQuat(local_Alink_des[linkId],&curQuat);

				outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 0 ] = local_link_pos_des[linkId][_X_];
				outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 1 ] = local_link_pos_des[linkId][_Y_];
				outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 2 ] = local_link_pos_des[linkId][_Z_];

				if (inEuler){
					quatToEulerXYZ(curQuat.q+1,eulerAngles);
					outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 3 ] = eulerAngles[0];
					outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 4 ] = eulerAngles[1];
					outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 5 ] = eulerAngles[2];
				}else{
					outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 3 ] = curQuat.q[_Q0_];
					outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 4 ] = curQuat.q[_Q1_];
					outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 5 ] = curQuat.q[_Q2_];
					outEFptr[timeStep*dimOut*N_ENDEFFS + (idx-1)*dimOut + 6 ] = curQuat.q[_Q3_];
				}
			}
		}
	}
	catch(char const* ex) { mexErrMsgTxt(ex); }
	catch(...) { mexErrMsgTxt("MexLib: computations failed"); }
}
