#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"
#include <math.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/sem.h>
#include <sys/types.h>
#include <string.h>

#include <errno.h>

#define NUM_USED_STATES 20

#include "SL_user.h"
#include "SL_episodic_communication.h"
#include "sharedmemory.h"
#include "sem_timedwait.h"


void mexFunction(int nlhs, mxArray *plhs[],	int nrhs, const mxArray *prhs[]) {
	static int episodeID    = 1;

	int i, j;
	if( attachEpisodicSharedMemoryCommunication() != -1 ) {

		int commandIdx      = *mxGetPr(prhs[0]);
		int maxCommands     = *mxGetPr(prhs[1]);
		double waitTime     = *mxGetPr(prhs[2]);
		int numStates       = mxGetNumberOfElements(prhs[4]);
		double *stateBuffer = mxGetPr(prhs[4]);
		int timeOut         = *mxGetPr(prhs[5]);

		timeOut *= 1000;

		setTimeOut(&signalState, timeOut);

		double * (joints[N_DOFS + 1]);

		double * jointsMatrix = mxGetPr(prhs[3]);

		int cols      = mxGetN(prhs[3]);
		int numSteps  = mxGetM(prhs[3]);

        
		if(cols != N_DOFS)
		{
			printf("Cols: %d, should be: %d\n", cols, N_DOFS);
			mexErrMsgTxt("Input trajectory  does not have correct number of joints\n.");
		}

		if(numSteps > STEPLIMIT)
		{
			printf("Steps: %d Step limit: %d\n", numSteps, STEPLIMIT);
			mexWarnMsgTxt("Input trajectory step size exceeds limit (STEPLIMIT) defined in ias_common.h. The trajectory will be cut. \n.");
			numSteps = STEPLIMIT;
		}

	
		for (j = 0; j < N_DOFS; ++j) {
			joints[j] = &(jointsMatrix[numSteps * j]);
		}


		setSemaphoreZero(&signalState) ;

		episodeTrajectory.episodeID         = episodeID;
		episodeTrajectory.trajectorySize          = numSteps;
		episodeTrajectory.commandIdx        = commandIdx;
		episodeTrajectory.waitingTime       = waitTime;
		episodeTrajectory.maxCommands          = maxCommands;


		//fprintf(stderr,"TrajIdx: %d, TrajSize %d\n",commandIdx, numSteps);


		if (numStates > 0) {
			for(j = 0; j < numStates; j++) {
				episodeTrajectory.stateBuffer[j] = stateBuffer[j];
			}
		}

		for(j = 0; j < N_DOFS; ++j) {
			for (i = 0; i < numSteps; ++i) {
				episodeTrajectory.trajectory[i + 1][j + 1] = joints[j][i];
			}
		}

		writeTrajectoryToSHM();

		getStateFromSHM();


	}

	plhs[0]             = mxCreateDoubleMatrix(NUMEPISODICSTATES, 1, mxREAL);
	plhs[1]             = mxCreateDoubleMatrix(1, 1, mxREAL);
	double *result      = mxGetPr(plhs[1]),	*state  = mxGetPr(plhs[0]);

	//     printf("stateID %d, episodeID %d\n",episodeState.episodeID , episodeID);
	if (episodeState.episodeID == episodeID) {
		for (i = 0; i < NUMEPISODICSTATES; ++i) {
			state[i] = episodeState.state[i];
		}
		*result = 1;
	} else {
		*result = -1;
	}

	episodeID = (episodeID + 1) % 2000;
	deleteEpisodicSharedMemoryCommunication();
}

