/***********************************************************************
 * Generic task to interface the real robot with Matlab using SL.
 * The user can use Matlab to:
 * - go to a desired position
 * - collect data with kinesthetic teaching
 * - execute a trajectory
 **********************************************************************/

#include "SL_system_headers.h"
#include "SL.h"
#include "SL_user.h"
#include "utility.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_man.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_kinematics.h"
#include "SL_filters.h"
#include "SL_shared_memory.h"
#include "SL_episodic_communication.h"
#include "SL_episodic_comm_utils.h"

#include "ias_utilities.h"
#include "ias_motor_utilities.h"

#include <errno.h>

#pragma message "N_DOFS value: " XSTR(N_DOFS)
#pragma message "STEPLIMITEPISODE value: " XSTR(STEPLIMITEPISODE)
#pragma message "N_DOFS_SHM value: " XSTR(N_DOFS_SHM)
#pragma message "STEPLIMITEPISODE value: " XSTR(STEPLIMITEPISODE)

// define the buffer indices for Matlab communication
#define COM_MATSTATE_IDX 0 // episodeTrajectory.stateBuffer[0] contains Matlab current state
#define COM_MAXTIME_IDX  1 // episodeTrajectory.stateBuffer[1] contains the max duration of the trajectory
#define COM_CTL_IDX      2 // episodeTrajectory.stateBuffer[2] contains the controller to use
#define COM_GOTO_IDX     0 // episodeState.state[0] will contain a flag indicating if the desired position can be reached or not

typedef enum {

	no_op         = 0,
	goto_pos      = 1,
	kinesthetic   = 2,
	execute_traj  = 3

} mat_state_t; // Matlab state, i.e. working mode


typedef enum {

	my_ctl        = -1,
	gravComp      = 0,
	invDyn        = 1,

} ctl_t; // controller to be used to execute a trajectory

static int isEpisodeOver;
static mat_state_t current_state;


/* Gets current buffer state. */
static inline mat_state_t getCurState () {
	
	return (int) (episodeTrajectory.stateBuffer[COM_MATSTATE_IDX] + 0.5); // round the state

}


/* Gets which controller to use from the buffer. */
static inline ctl_t getCtl () {
	
	return (int) (episodeTrajectory.stateBuffer[COM_CTL_IDX] + 0.5); // round the state

}


/* Full model-based controller. */
static void myFullInvDyn () {

#ifdef DARIAS

	double massMatrix[8][8]; // TODO for the left arm

	massMatrix[1][1] = misc_sensor[R_IM11];
	massMatrix[1][2] = misc_sensor[R_IM12];
	massMatrix[1][3] = misc_sensor[R_IM13];
	massMatrix[1][4] = misc_sensor[R_IM14];
	massMatrix[1][5] = misc_sensor[R_IM15];
	massMatrix[1][6] = misc_sensor[R_IM16];
	massMatrix[1][7] = misc_sensor[R_IM17];

	massMatrix[2][1] = misc_sensor[R_IM12];
	massMatrix[2][2] = misc_sensor[R_IM22];
	massMatrix[2][3] = misc_sensor[R_IM23];
	massMatrix[2][4] = misc_sensor[R_IM24];
	massMatrix[2][5] = misc_sensor[R_IM25];
	massMatrix[2][6] = misc_sensor[R_IM26];
	massMatrix[2][7] = misc_sensor[R_IM27];

	massMatrix[3][1] = misc_sensor[R_IM13];
	massMatrix[3][2] = misc_sensor[R_IM23];
	massMatrix[3][3] = misc_sensor[R_IM33];
	massMatrix[3][4] = misc_sensor[R_IM34];
	massMatrix[3][5] = misc_sensor[R_IM35];
	massMatrix[3][6] = misc_sensor[R_IM36];
	massMatrix[3][7] = misc_sensor[R_IM37];

	massMatrix[4][1] = misc_sensor[R_IM14];
	massMatrix[4][2] = misc_sensor[R_IM24];
	massMatrix[4][3] = misc_sensor[R_IM34];
	massMatrix[4][4] = misc_sensor[R_IM44];
	massMatrix[4][5] = misc_sensor[R_IM45];
	massMatrix[4][6] = misc_sensor[R_IM46];
	massMatrix[4][7] = misc_sensor[R_IM47];

	massMatrix[5][1] = misc_sensor[R_IM15];
	massMatrix[5][2] = misc_sensor[R_IM25];
	massMatrix[5][3] = misc_sensor[R_IM35];
	massMatrix[5][4] = misc_sensor[R_IM45];
	massMatrix[5][5] = misc_sensor[R_IM55];
	massMatrix[5][6] = misc_sensor[R_IM56];
	massMatrix[5][7] = misc_sensor[R_IM57];

	massMatrix[6][1] = misc_sensor[R_IM16];
	massMatrix[6][2] = misc_sensor[R_IM26];
	massMatrix[6][3] = misc_sensor[R_IM36];
	massMatrix[6][4] = misc_sensor[R_IM46];
	massMatrix[6][5] = misc_sensor[R_IM56];
	massMatrix[6][6] = misc_sensor[R_IM66];
	massMatrix[6][7] = misc_sensor[R_IM67];

	massMatrix[7][1] = misc_sensor[R_IM17];
	massMatrix[7][2] = misc_sensor[R_IM27];
	massMatrix[7][3] = misc_sensor[R_IM37];
	massMatrix[7][4] = misc_sensor[R_IM47];
	massMatrix[7][5] = misc_sensor[R_IM57];
	massMatrix[7][6] = misc_sensor[R_IM67];
	massMatrix[7][7] = misc_sensor[R_IM77];

	if ( real_robot_flag ) {
		int i, j;
		for (i = 1; i <= 7; ++i) {
			for (j = 1; j <= 7; ++j)
				joint_des_state[i].uff += massMatrix[i][j] * joint_des_state[j].thdd;
		}
	}
	else
		SL_InvDyn( joint_state, joint_des_state, endeff, &base_state, &base_orient );

#else

	SL_InvDyn( joint_state, joint_des_state, endeff, &base_state, &base_orient );

#endif

}


/* PD with gravity compensation controller. */
static void myGravComp () {

#ifdef DARIAS

	if ( ! real_robot_flag )
		SL_InvDyn( 0, joint_des_state, endeff, &base_state, &base_orient );

#else

	SL_InvDyn( 0, joint_des_state, endeff, &base_state, &base_orient );

#endif

}


/* Executes one step of the episode. */
static void doMotionEpisodeStep(int commIdx, double waitForMatlab, int step) {

	current_state = getCurState(); // get Matlab current state, i.e. what action Matlab requires

	int i;
	switch(current_state) {

		case goto_pos: { // if goto position...

			if ( step == 1 )
				initGotoPosMinJerkSL(2.0, TRUE); // ...initialize the goto procedure (2.0 is the time to reach to position)...

			SL_DJstate gotoPos[N_DOFS+1];

			for ( i = 1; i <= N_DOFS; ++i ) { // ...then take the position from the shared memory...
				gotoPos[i].th   = episodeTrajectory.trajectory[1][i];
				gotoPos[i].thd  = 0.0;
				gotoPos[i].thdd = 0.0;
			}

			episodeState.state[COM_GOTO_IDX] = gotoPosMinJerkSL(gotoPos); // ... goto position with min jerk joint configuration and write in the buffer if everything went well

			break;

		}

		case kinesthetic: { // if you have to do kinesthetic teaching...

			for ( i = 1; i <= N_DOFS; ++i ) { // ...set the desired state as the current state
				joint_des_state[i].th  = joint_state[i].th;
				joint_des_state[i].thd = 0.99 * joint_state[i].thd;
			}

			// do not do anything else: all the data will be automatically saved and can be retrieved by SLGetEpisode in Matlab

			break;

		}

		case execute_traj: { // if you have to execute a trajectory...
			
			static int startMovementStep = 0;

			if (step == 1) {
				isEpisodeOver = 0;
				startMovementStep = (int) (episodeTrajectory.waitingTime * task_servo_rate + 0.5); // ...calculate when to start, according to the waiting time passed by Matlab
			}

			// wait the amount of time indicated by Matlab ('episodeTrajectory.waitingTime')...
			if (step >= startMovementStep && (step - startMovementStep <= episodeTrajectory.trajectorySize)) {
				for (i = 1; i <= N_DOFS; i++) {
					joint_des_state[i].th = episodeTrajectory.trajectory[step - startMovementStep][i];
					joint_des_state[i].thd = 0;
					joint_des_state[i].thdd = 0;
				}
			}

			// if the trajectory is over, set the global variable used by 'isStepOver' and let the robot keep the current position
			if ((step - startMovementStep) > episodeTrajectory.trajectorySize) {
				isEpisodeOver = 1;
			}

			break;

		}

		default:

			break;

	}

}


/* Checks if the episode is over. */
static int isStepOver(int commIdx, double waitForMatlab, int step) {

	int i;

	if (commIdx == 1)
		return TRUE; // always return 1 when receiving SLInitEpisode

	// if the robot has reached the max time indicated by Matlab, then stop
	if ( step + 1 >=
			episodeTrajectory.trajectorySize + (episodeTrajectory.stateBuffer[COM_MAXTIME_IDX] * task_servo_rate) ) {
		current_state = no_op;
		printf("Time is over\n");

		return 1;
	}

	episodeState.commandIdx = commIdx;
	episodeState.episodeID 	= episodeTrajectory.episodeID;

	if (current_state == goto_pos ) { // if you had to accomplish a 'goto' task...
		if (episodeState.state[COM_GOTO_IDX]) { // ...return if the final position was reached or not
			current_state = no_op;
			printf("Goal position reached\n");

			for (i = 1; i <= N_DOFS; i++) {
				joint_des_state[i].th = joint_state[i].th;
			}

			return 1;
		}
	}

	if (current_state == execute_traj) { // if you had to execute a trajectory...
		if (isEpisodeOver) { // ...return if the trajectory is fully executed
			current_state == no_op;
			printf("Trajectory is over\n");

			for (i = 1; i <= N_DOFS; i++) {
				joint_des_state[i].th = joint_state[i].th;
			}

			return 1;
		}
	}

	// for the kinesthetic teaching, the function will return 1 only if the first condition is triggered, i.e., if the time is over

	return 0;
}


/* Writes in the shared memory at each time step. The array it writes into
 * is the one named 'episodeState' in the function 'SLGetEpisodeMex'. */
static void writeStepToSharedMemory(int curStepInEpisode) {

	writeStepToSharedMemoryGeneral(curStepInEpisode);

}


static int init_gen_matlab_task(void) {

	// check whether any other task is running
	if (strcmp(current_task_name,NO_TASK) != 0) {
		printf("Task can only be run if no other task is running!\n");
		return FALSE;
	}

	// open the communication with Matlab
	createEpisodicSharedMemoryCommunication();

	// clear the graphics
	sendUserGraphics("clearUserGraphics",0,0);

	// useInverseDynamics, pGain, dGain, initPos, gotoPosThreshold
	initMotorUtilities(1, 40, 10, joint_default_state, 0.001);

	// set the time the robot takes for 'gotoInitPosSL', called by SLResetEpisode	
	t_ratio = 0.5; 

	if (!go_target_wait_ID(joint_default_state))
		return FALSE;

	current_state = no_op;

	// ready to go
	int ans = 999;
	while ( ans == 999 ) {
		get_int("Enter 1 to start or anthing else to abort ...",ans,&ans);
	}

  	if ( ans != 1 ) {
		return FALSE;
	}

	return TRUE;
}


static int run_gen_matlab_task(void) {

	int i;
	for ( i = 1; i <= N_DOFS; ++i ) {
		joint_des_state[i].thd  = 0.0;
		joint_des_state[i].thdd = 0.0;
	}

	int simPhysics = FALSE;

	// this line calls (in the following order): doMotionEpisodeStep, isStepOver, writeStepToSharedMemory
	episodicSharedMemoryWait(&doMotionEpisodeStep, &isStepOver, &writeStepToSharedMemory, &simPhysics, !real_robot_flag, 1);
	// the second to last argument corresponds to 'waitForMatlab': if 1, SL will always wait for Matlab commands and won't do anything else,
	// if 0, after ending the episode sent by Matlab it will temporarily freeze

	switch ( getCtl() ) {

		case my_ctl: {

			break;

		}

		case gravComp: {

			inverseDynControl = 0;
			myGravComp();
			break;

		}

		case invDyn: {

			inverseDynControl = 1;
			myFullInvDyn();
			break;

		}

		default: {

			break;

		}
	}

	check_range(joint_des_state);
	
	return TRUE;

}


static int change_gen_matlab_task(void) {

	return TRUE;

}


void add_gen_matlab_task( void ) {

	addTask("Generic Matlab Task", init_gen_matlab_task,run_gen_matlab_task, change_gen_matlab_task);

}
