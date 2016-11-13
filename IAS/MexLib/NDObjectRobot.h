/*
 * NDObjectRobot.h
 *
 *  Created on: Mar 29, 2014
 *      Author: rueckert
 */

#ifndef NDOBJECTROBOT_H_
#define NDOBJECTROBOT_H_

#include "NumericDerivationObject.h"
#include "SL_system_headers.h"
#include "SL.h"
#include "SL_dynamics.h"
#include "SL_user.h"
#include "pthread.h"
#include "utility.h"
#include "SL_kinematics.h"
#include "SL_common.h"

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

class NDObjectRobot : public NumericDerivationObject {
private:
	int n_dofs_;
	double *endeffectorPosAndOrients_;//6*N_ENDEFFS array

public:
	NDObjectRobot(double *x, double *y, int szX, int szU) 
: NumericDerivationObject(x, y, szX + szU, szX), n_dofs_(szU),
  endeffectorPosAndOrients_(NULL) { }

	NDObjectRobot(double *x, double *y, int szX, int szU, double *endeffectorPosAndOrients )
	: NumericDerivationObject(x, y, szX + szU, szX), n_dofs_(szU),
	  endeffectorPosAndOrients_(endeffectorPosAndOrients) { }

	NDObjectRobot(const NDObjectRobot& cpyObj)
	: NumericDerivationObject(cpyObj), n_dofs_(cpyObj.getDofs()),
	  endeffectorPosAndOrients_(NULL) { }

	virtual ~NDObjectRobot() {
	}

	virtual void * RunThread() {

		int slStartIndex = 1;
		int i, posAndOrientId;
		SL_Jstate jts[n_dofs_+1];
		SL_uext   ux[n_dofs_+1];
		bzero((void *)jts,sizeof(SL_Jstate)* (n_dofs_+1) );
		bzero((void *)ux,sizeof(SL_uext)* (n_dofs_+1) );
		// set the current state
		for(i=0; i < n_dofs_; i++) {
			jts[i+slStartIndex].th = x_[i];
			jts[i+slStartIndex].thd = x_[n_dofs_ + i];
			jts[i+slStartIndex].u = x_[2*n_dofs_ + i];
		}

		pthread_mutex_lock(&mutex);

		//		SL_ForDyn(jts, &base_state_cpy_, &base_orient_cpy_, ux, endeff_cpy_);
		SL_ForDyn(jts, &base_state, &base_orient, ux, endeff);

		// Articulated Body Dynamics forward dynamics
		//SL_ForDynArt(jts, bs, bo, ux, endeff);
		// Comp.Inertia forward dynamics
		//SL_ForDynComp(jts, &bs, &bo, ux, endeff, NULL, NULL);
		pthread_mutex_unlock(&mutex);


		//save the results
		for (i=0; i < n_dofs_; i++) {
			y_[i] = x_[i+ n_dofs_];
			y_[i+n_dofs_] = jts[i+slStartIndex].thdd;
		}

		if(endeffectorPosAndOrients_ != 0) {

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
				state[i+slStartIndex].th = x_[i];
				state[i+slStartIndex].thd = x_[N_DOFS + i];
				state[i+slStartIndex].uff = x_[2*N_DOFS + i];
			}

			linkInformationDes(state,&base_state,&base_orient,endeff,
					local_joint_cog_mpos_des,
					local_joint_axis_pos_des,
					local_joint_origin_pos_des,
					local_link_pos_des,
					local_Alink_des,
					local_Ahmatdof);

			int endEffId, linkId;
			SL_quat curQuat;
			Vector orientations = my_vector(1,4);

			for(endEffId=0; endEffId < N_ENDEFFS; endEffId++){
				linkId = link2endeffmap[endEffId + slStartIndex]-1;
				//linkQuat(local_Alink_des[linkId+slStartIndex],&curQuat);
				//quatToEuler(&curQuat, orientations);
				rotMatToEuler(local_Alink_des[linkId+slStartIndex], orientations);

				//				printf("Link [%d] ", linkId);
				for(int posId = 0; posId < 3; posId++) {
					//					printf("p: %f o: %f ",
					//							local_joint_origin_pos_des[linkId+slStartIndex][posId+slStartIndex],
					//							orientations[posId+slStartIndex]);
					endeffectorPosAndOrients_[ endEffId*6 + posId ] =
							local_joint_origin_pos_des[linkId+slStartIndex][posId+slStartIndex];
					endeffectorPosAndOrients_[ endEffId*6 + 3 + posId ] =
							orientations[posId+slStartIndex];
				}
				//				printf("\n");
			}

		}

		return NULL;
	}

	int getDofs() const {
		return n_dofs_;
	}

	double* getEndeffectorPosAndOrients() const {
		return endeffectorPosAndOrients_;
	}


	double getEndeffectorPosAndOrients(int index) const {
		return endeffectorPosAndOrients_[index];
	}

	void setEndeffectorPosAndOrients(double* endeffectorPosAndOrients) {
		endeffectorPosAndOrients_ = endeffectorPosAndOrients;
	}
};

#endif /* NDOBJECTROBOT_H_ */
