#include "SL_episodic_comm_utils.h"

#include "SL.h"
#include "SL_task_servo.h"
#include "ias_motor_utilities.h"


#include <stdio.h>
#include <string.h>



int state;
int maxCommands;
int useInitStep = 1;
int commandIdx;

static int goToInitialPos = 1;

void writeStepToSharedMemoryGeneral(int numStepInEpisode)
{
	episodeStep.episodeID = episodeTrajectory.episodeID;
	if (numStepInEpisode >= STEPLIMITEPISODE)
	{
		numStepInEpisode = STEPLIMITEPISODE - 1;
	}
	episodeStep.numTransmittedSteps = numStepInEpisode;
	episodeStep.commandIdx[numStepInEpisode] = commandIdx;
	episodeStep.stepInTrajectory[numStepInEpisode] = stepInTrajectory;
	int i = 0;
	for (i = 1; i <= N_DOFS_SHM; i ++)
	{
		episodeStep.joints[numStepInEpisode][i-1] = joint_state[i].th;
		episodeStep.jointsVel[numStepInEpisode][i-1] = joint_state[i].thd;
		episodeStep.jointsAcc[numStepInEpisode][i-1] = joint_state[i].thdd;

		episodeStep.jointsDes[numStepInEpisode][i-1] 	= joint_des_state[i].th;
		episodeStep.jointsVelDes[numStepInEpisode][i-1] = joint_des_state[i].thd;
		episodeStep.jointsAccDes[numStepInEpisode][i-1] = joint_des_state[i].thdd;


		episodeStep.torque[numStepInEpisode][i-1] = joint_state[i].u;

	}
//	printf("%d : %f %f %f\n", episodeStep.numStepsEpisode, episodeStep.joints[numStepInEpisode][0], episodeStep.joints[numStepInEpisode][1], episodeStep.joints[numStepInEpisode][2]);
	for (i = 1; i <= 3; i ++)
	{
		episodeStep.cart[numStepInEpisode][i - 1] = cart_state[1].x[i];
	}

	for (i = 1; i <= 4; i ++)
	{
		episodeStep.cart[numStepInEpisode][i - 1 + 3] = cart_orient[1].q[i];
	}

}

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
// Replay Episodes Using Matlab
void episodicSharedMemoryWait(void (*doMotionEpisodeStepFunc)(int, double, int),
int (*isStepOverFunc)(int , double , int ), void (*writeStepToSharedMemoryFunc)(int ),
 int *simPhysics, int waitForMatlab, int holdInPlace)
{
	static int step;

	static double sendDataTime;
	static double gotDataTime;
	static double newStepTime;
	static double gotoStartTime;


	static int firsttime   			= TRUE;
	static int hitStart 			= FALSE;
	static int gotoStartFirstTime 	= FALSE;
	static int gotoStartSteps = 0;
	static int numStepInEpisode = 0;

	static double maxJointDiff = 0.03;
	int i = 0;
	/////////////////////////////////////////////////////
	// INITIALIZE
	if(firsttime)
	{
		*simPhysics     = FALSE;
		firsttime       = FALSE;
		state           = RESTARTEPISODE;
		gotoStartTime   = servo_time;
		commandIdx = 0;


	}

	if ( state == RESTARTEPISODE )
	{
		writeSHMemory(&stepSHM, &episodeStep, &stepSEM);
		state 			= GOTOSTART;
		numStepInEpisode 	= 0;
		firstGotoPos 		= 1;
		gotoStartSteps 		= 0;
	}
	*simPhysics = FALSE;
	if( state == GOTOSTART )
	{
		double kPosTmp = kPos;
		kPos = 40;
		if((holdInPlace && commandIdx > 0) || !goToInitialPos)
		{
			sendDataTime 	= servo_time;
			commandIdx 	= 0;
			state 		= WAITFORSTART;
			firstGotoPos 	= 1;
		}
		else
		{
			*simPhysics  	= FALSE;
			commandIdx 	= 0;
			gotoStartSteps ++;
			int result 	= gotoPosMinJerkSL( init_posture );

			//printf("GotoStart %d, result %d, %f %f %f %f %f %f %f \n", firstGotoPos, result, init_posture[1].th,
			//	init_posture[2].th, init_posture[3].th, init_posture[4].th, init_posture[5].th, init_posture[6].th, init_posture[7].th);


			if(result == 1 && gotoStartSteps > 1500) // 1500
			{
				//~ printf("Reached Start Position\n");
				//~ for (i = 1; i < N_DOFS; i++)
				//~ {
					//~ printf("%f %f %f, ", initPosture[i], rest_posture[i].th, joint_state[i].th);
					//~
				//~ }
				sendDataTime 	= servo_time;
				state 			= WAITFORSTART;
				firstGotoPos 	= 1;

			}
			else
			{
				gotoStartFirstTime 	= TRUE;
				gotoStartTime 		= servo_time;
			}
		}
		kPos = kPosTmp;
	}

	if ( state == WAITFORSTART )
	{
		if(!holdInPlace && goToInitialPos)
		{
			gotoPosMinJerkSL( init_posture);
		}
		int result;
		result = isStartEpisodeStepWait(commandIdx + 1, waitForMatlab);

		if(result > 0)
		{

			maxCommands = episodeTrajectory.maxCommands;

			step		= 1;
			commandIdx 	= commandIdx + 1;
			DEBUGPRINT("Start Command %d\n", commandIdx);
			if (maxCommands == 1)
			{
				sendDataTime 	= servo_time;
				gotoStartTime 	= servo_time;

				memset(&episodeStep, 0, sizeof(episode_steps));
			}
			state       	= DOMOTION;
			firstGotoPos 	= 1;

			gotDataTime = servo_time;
			//printf("State: DOMOTION %d %f %f : %f \n", gameStage, gotDataTime
			//- gotoStartTime, gotDataTime - sendDataTime, episodeTrajectory.waitingTime);
			/*for (i = 0; i < 20; i ++)
			{
					printf("%f %f %f %f %f %f %f\n", episodeTrajectory.trajectory[i][1], episodeTrajectory.trajectory[i][2], episodeTrajectory.trajectory[i][3], episodeTrajectory.trajectory[i][4], episodeTrajectory.trajectory[i][5], episodeTrajectory.trajectory[i][6], episodeTrajectory.trajectory[i][7] );
			}*/
		}
		if (result == -666) {
			printf("Episode was finished manually.\n");
			writeStateToSHM();
			commandIdx = 0;
			state = RESTARTEPISODE;
			writeStateToSHM();
		}

		if (result == -100) {
			printf("Error : Got Wrong Episode Step %d %d\n", commandIdx +1 , episodeTrajectory.commandIdx);
			commandIdx = 0;
			state = RESTARTEPISODE;
			//writeStateToSHM();
			//~ gotoStartSteps = 0;
		}

		if (result == 0 && servo_time - sendDataTime > MAX_STEP_TIME && commandIdx > 0) {
			if (commandIdx > 1)
			{
				printf("TIMEOUT in state %d : %f\n", commandIdx, servo_time - sendDataTime);
			}
			state = RESTARTEPISODE;

			//~ gotoStartSteps = 0;
		}

		if (commandIdx > 1 ) {
		    if (useInitStep == 0 )
		    {
		    	writeStepToSharedMemoryFunc(numStepInEpisode);
		    	numStepInEpisode ++;
		    }
		    *simPhysics  = TRUE;
		}
	}

	if( state == DOMOTION )
	{

		doMotionEpisodeStepFunc(commandIdx, sendDataTime, step);
		step = step + 1;
		int result = isStepOverFunc(commandIdx, gotDataTime, step);
//		printf("result %d\n", result);

		if (result)
		{
			DEBUGPRINT("Step %d is over : %f Writing to SHM\n", gameStage, servo_time - gotoStartTime);
			writeSHMemory(&stepSHM, &episodeStep, &stepSEM);
			writeStateToSHM();

			sendDataTime 	= servo_time;
			if (commandIdx == maxCommands)
			{
				state = RESTARTEPISODE;
			}
			else
			{
				state 		= WAITFORSTART;
				firstGotoPos 	= 1;
			}
		}

		if (result == -1)
		{
			state = RESTARTEPISODE;
		}

		if (useInitStep == 0 || commandIdx >= 2)
		{
		    writeStepToSharedMemoryFunc(numStepInEpisode);
		    *simPhysics  = TRUE;
		    numStepInEpisode ++;
		}
	}
}

void episodicSharedMemory(void (*doMotionEpisodeStepFunc)(int, double, int),
	int (*isStepOverFunc)(int , double , int ), void (*writeStepToSharedMemoryFunc)(int ),
	int *simPhysics)
{
	return episodicSharedMemoryWait(doMotionEpisodeStepFunc, isStepOverFunc,
		writeStepToSharedMemoryFunc, simPhysics, 0, 0);
}

void episodicSharedMemoryNoMotion(void (*doMotionEpisodeStepFunc)(int, double, int),
	int (*isStepOverFunc)(int , double , int ), void (*writeStepToSharedMemoryFunc)(int ),
	int *simPhysics)
{
	goToInitialPos = 0;
	return episodicSharedMemoryWait(doMotionEpisodeStepFunc, isStepOverFunc,
		writeStepToSharedMemoryFunc, simPhysics, 0, 0);
}
