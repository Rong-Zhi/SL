#include "SL_system_headers.h"

#include "SL.h"
#include "SL_user.h"
#include "utility.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_man.h"
#include "SL_dynamics.h"

#include "ias_utilities.h"
#include "ias_motor_utilities.h"
#include "SL_episodic_communication.h"
#include "SL_episodic_comm_utils.h"

#include <errno.h>


typedef enum {

	goto_pos      = 0,
	execute        = 1

}  state_t;


#define IN_STATE 0
#define IN_CTL_DURATION 1

#define OUT_GOTO_POS_AT_INIT_POS 0

static int useInvDyn = 1;
static int startMovementStep = 0;


void doMotionEpisodeStep_MatCtl (int idxTraj, double sendDataTime, int step) {

	state_t state = (int) (episodeTrajectory.stateBuffer[IN_STATE]+0.5);

	switch(state){

	case goto_pos: {

		if ( step == 1 )
			initGotoPosMinJerkSL(1.0, TRUE);

		SL_DJstate gotoPos[N_DOFS+1];

		int i;
		for ( i = 1; i <= N_DOFS; ++i ) {
			gotoPos[i].th   = episodeTrajectory.trajectory[1][i];
			gotoPos[i].thd  = 0.0;
			gotoPos[i].thdd = 0.0;
		}

		episodeState.state[OUT_GOTO_POS_AT_INIT_POS] = gotoPosMinJerkSL(gotoPos);

		break;

	}

	case execute: {

		static int startMovement = FALSE;
		static int task_start_time = 0;


		if (step == 1) {
			startMovementStep = 0;
			task_start_time = step;
			startMovement = FALSE;
		}

		if ((step-task_start_time) > (int)((episodeTrajectory.waitingTime * task_servo_rate )+0.5) && startMovement == FALSE) {
			startMovement = TRUE;
			startMovementStep = step;
		}

		int ctl_duration = (int) (episodeTrajectory.stateBuffer[IN_CTL_DURATION]+0.5);

		if (startMovement == TRUE && ((step - startMovementStep) <= (episodeTrajectory.trajectorySize/(2*N_DOFS+1)*ctl_duration   ) )) {


			doControl( episodeTrajectory.trajectory,  step - startMovementStep, ctl_duration,  episodeTrajectory.trajectorySize);

		}
		else if ( startMovement == TRUE ) {

			int i;
			for ( i = 1; i <= N_DOFS; ++i ) {

					joint_des_state[i].th = joint_state[i].th;  //Resetting the internal PD //TODO this  is drifting
					joint_des_state[i].thd = 0;
					joint_des_state[i].thdd = 0;
			}
		}


//		if( (step - startMovementStep) > episodeTrajectory.trajectorySize) {
//
//			if (step - startMovementStep  == episodeTrajectory.trajectorySize + 1)
//				initGotoPosMinJerkSL(1.0, TRUE);
//
//			gotoPosMinJerkSL(init_posture);  //TODO I guess delete that
//
//		}

		break;

	}

	default:
		break;

	}

}




int isStepOver_MatCtl (int idxTraj, double sendDataTime, int step){

	state_t state = (int) (episodeTrajectory.stateBuffer[IN_STATE]+0.5);

	episodeState.commandIdx = idxTraj;
	episodeState.episodeID 	= episodeTrajectory.episodeID;



	switch(state){

	case goto_pos: {
		//Storing whether the position was reached or not at doMotionEpisodeStep

		return episodeState.state[OUT_GOTO_POS_AT_INIT_POS];
	}

	case execute: {
		int ctl_duration = (int) (episodeTrajectory.stateBuffer[IN_CTL_DURATION]+0.5);
		int maxStep =  episodeTrajectory.trajectorySize / (2*N_DOFS+1) * ctl_duration + 500;
		return (step - startMovementStep >= maxStep);
	}


	default:
		return 1;

	}

}



void writeStepToSharedMemory_MatCtl (int curStepInEpisode) {

	state_t state = (int) (episodeTrajectory.stateBuffer[IN_STATE]+0.5);

	static int firstStep = -1;

	if ( state != execute ) {
		firstStep = -1;
		return;
	}

	if ( firstStep == -1 )
		firstStep = curStepInEpisode;

	curStepInEpisode = curStepInEpisode - firstStep;

	writeStepToSharedMemoryGeneral(curStepInEpisode);

}




static int init_ctlMatlab_Task(void) {

	if (strcmp(current_task_name,NO_TASK) != 0) {
		printf("Task can only be run if no other task is running!\n");
		return FALSE;
	}

	createEpisodicSharedMemoryCommunication();


//	SL_DJstate  goto_state[N_DOFS+1];
//
//	goto_state[1].th = -0.8;
//	goto_state[2].th = 0.0;
//	goto_state[3].th = -0.25;
//	goto_state[4].th = 0.6;
//	goto_state[5].th = 0.0;
//	goto_state[6].th = 0.4;
//	goto_state[7].th = -0.4;
//	goto_state[8].th = 0.0;
//	goto_state[9].th = 0;
//
//	if (!go_target_wait_ID(goto_state))
//		return FALSE;


	get_int("Enable Inverse Dynamics?", useInvDyn, &(useInvDyn));
	useInvDyn = useInvDyn ? 1 : 0;

	// useInverseDynamics, pGain, dGain, initPos, gotoPosThreshold
	initMotorUtilities(useInvDyn, 40, 10, joint_default_state, 0.001);

	return TRUE;
}


static int run_ctlMatlab_Task(void) {

	int simPhysics  = FALSE;
	episodicSharedMemoryWait(&doMotionEpisodeStep_MatCtl, &isStepOver_MatCtl,&writeStepToSharedMemory_MatCtl, &simPhysics, !real_robot_flag, 0);

	if ( useInvDyn )
		SL_InvDyn(joint_state,joint_des_state,endeff, &base_state, &base_orient);

	return TRUE;

}


static int change_ctlMatlab_Task(void) {
	return TRUE;
}


void add_genMatlab_Task( void ) {
	addTask("Ctl Matlab Task", init_ctlMatlab_Task, run_ctlMatlab_Task, change_ctlMatlab_Task);
}
