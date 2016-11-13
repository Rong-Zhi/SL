/*
 * FwdDynRobot.h
 *
 *  Created on: Mar 29, 2014
 *      Author: rueckert
 */

#ifndef FWDDYNANDKINROBOT_H_
#define FWDDYNANDKINROBOT_H_

#include "FwdDynRobot.h"
#include "SL.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "SL_user.h"
#include "utility.h"
#include "SL_kinematics.h"

//using namespace FwdDynRobot;
#define MSG_EF_JACOBIANS_ARRAY_UNINITIALIZED "MexLib: JACOBIAN output array is a zero pointer";

template <class T>
class FwdDynAndKinRobot : public FwdDynRobot<T> {
protected:
	double* outArrayPtrEFJacobians_;

public:
	FwdDynAndKinRobot(double *x, double *u, int szX, int szU,
			double* outArrayPtrXdot, double* outArrayPtrXdotX,
			double* outArrayPtrXdotU, double* outArrayPtrEFJacobians)
: FwdDynRobot<T>(x, u, szX, szU,outArrayPtrXdot, outArrayPtrXdotX, outArrayPtrXdotU),
  outArrayPtrEFJacobians_(outArrayPtrEFJacobians) { }

	~FwdDynAndKinRobot() {}

	int initiateComputation() {

		int retVal = 0;
		int szX = FwdDynRobot<T>::getSzX();
		int szU = FwdDynRobot<T>::getSzU();
		int szX2 = szX*szX;
		double *x = FwdDynRobot<T>::getX();
		double *u = FwdDynRobot<T>::getU();
		double *xdot = FwdDynRobot<T>::getOutArrayPtrXdot();
		double *xdotx = FwdDynRobot<T>::getOutArrayPtrXdotX();
		double *xdotu = FwdDynRobot<T>::getOutArrayPtrXdotU();

		if (!init_dynamics())
			return NULL;

		double xAndU[szX + szU];
		int i;
		for(i=0; i < szX; i++)
			xAndU[i] = x[i];
		for(i=0; i< szU; i++)
			xAndU[szX+i] = u[i];

		NDObjectRobot single(xAndU, xdot, szX, szU, outArrayPtrEFJacobians_);

		double resultsYdotX[szX*(szX + szU)];
		if(1) {
			retVal = NumericDerivation<T>::initiatePartitialDerivative(&single, resultsYdotX);
			computeAnalyticJacobian();
		} else {
			double resultsEFdotX[6*N_ENDEFFS* (szX + szU)];
			retVal = NumericDerivation<T>::initiatePartitialDerivative(&single,
					resultsYdotX, resultsEFdotX);
			//computeNumericJacobian();
			for(i=0; i < 6*N_ENDEFFS*szX; i++)
				outArrayPtrEFJacobians_[i] = resultsEFdotX[i];
		}
		for(i=0; i< szX2; i++)
			xdotx[i] = resultsYdotX[i];
		for(i=0; i< szX * szU; i++)
			xdotu[i] = resultsYdotX[szX2 + i];

		return retVal;
	}

	void computeAnalyticJacobian() {

		if(outArrayPtrEFJacobians_==0)
			throw MSG_EF_JACOBIANS_ARRAY_UNINITIALIZED;

		init_kinematics();

		int i;
		int slStartIndex = 1;
		Matrix fullJ=my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
		Matrix local_link_pos_des         = my_matrix(0,N_LINKS,1,3);
		Matrix local_joint_cog_mpos_des   = my_matrix(0,N_DOFS,1,3);
		Matrix local_joint_origin_pos_des = my_matrix(0,N_DOFS,1,3);
		Matrix local_joint_axis_pos_des   = my_matrix(0,N_DOFS,1,3);
		Matrix local_Alink_des[N_LINKS+1];
		Matrix local_Ahmatdof[N_DOFS+1];

		for (i=0; i<=N_LINKS; ++i)
			local_Alink_des[i] = my_matrix(1,4,1,4);
		SL_DJstate state[N_DOFS+1];
		bzero((void *)state,sizeof(SL_DJstate)* (N_DOFS+1) );
		// set the current state
		for(i=0; i < N_DOFS; i++) {
			state[i+slStartIndex].th = FwdDynRobot<T>::x_[i];
			state[i+slStartIndex].thd = FwdDynRobot<T>::x_[N_DOFS + i];
			state[i+slStartIndex].uff = FwdDynRobot<T>::u_[i];
		}

		linkInformationDes(state,&base_state,&base_orient,endeff,
				local_joint_cog_mpos_des,
				local_joint_axis_pos_des,
				local_joint_origin_pos_des,
				local_link_pos_des,
				local_Alink_des,
				local_Ahmatdof);

		jacobian(local_link_pos_des,local_joint_origin_pos_des,local_joint_axis_pos_des,fullJ);

		//TODO use memcopy
		int numElementsToCopy = 6*N_ENDEFFS*N_DOFS;
		//velocity jacobian set to zero
		for(i=0; i < numElementsToCopy*2; i++)
			outArrayPtrEFJacobians_[i] = 0.0;
		//memcpy( (void*)dst, (void*)src, n * sizeof(int) );

		int rowId, colId, mxArrayId;
		for(colId=0; colId < N_DOFS; colId++) {
			for(rowId=0; rowId < 6*N_ENDEFFS; rowId++) {
				mxArrayId = colId*(6*N_ENDEFFS)+rowId;
				outArrayPtrEFJacobians_[mxArrayId] = fullJ[rowId+slStartIndex][colId+slStartIndex];
			}
		}
	}

};

#endif /* FWDDYNANDKINROBOT_H_ */
