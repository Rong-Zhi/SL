/***********************************************************************
 * This file contains the main functions for the ball on a beam task, 
 * describes the reward function and which controller to use.
 **********************************************************************/

/* global includes */
#include "SL_system_headers.h"
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_dynamics.h"
#include "SL_episodic_comm_utils.h"
#include "ias_motor_utilities.h"

/* local includes */
#include "ballonbeam.h"

/* global functions */
void add_ballonbeam_task(void);

/* local functions */
static int initBallOnBeamTask(void);
static int runBallOnBeamTask(void);
static int changeBallOnBeamTask(void);
static double calc_reward_ballonbeam(void);

static int numTrials;

static double k = 0.f; //025,
static double d = 0.f; //017,
static double a = 0.f; //01,

static double total_reward = 0.0;
static double u = 0;
static double wait_for_start;
static SL_DJstate init_joint_state[N_DOFS+1];
static double start_time = 0;
static double ending_time = 10;


/* Writes in the shared memory at each time step. The array it writes into
 * is the one named 'state' in the function 'SLGetEpisodeMex'. */
static void writeStepToSharedMemory(int numStepInEpisode)
{
	writeStepToSharedMemoryGeneral(numStepInEpisode);
	episodeStep.state[numStepInEpisode][0] = calc_reward_ballonbeam();
	episodeStep.state[numStepInEpisode][1] = ballState.x[1];
	episodeStep.state[numStepInEpisode][2] = ballState.xd[1];
}

/* Execute one step of the episode. */
static void doMotionEpisodeStep(int numCommand, double waitForStart, int step)
{
	int i = 0;
	
	wait_for_start = waitForStart;

	// command 1: go to init position (ball position passed in the buffer) (Matlab called 'SLInitEpisode')
	if (numCommand == 1)
	{
		resetBallOnBeamSimulation();
	  	ballState.x[_X_] 	= episodeTrajectory.stateBuffer[0];
		ballState.xd[_X_] 	= episodeTrajectory.stateBuffer[1];
		ballState.xdd[_X_] 	= episodeTrajectory.stateBuffer[2];
		
		gotoPosMinJerkSL( init_posture );
	}
	// command 2: use a controller (Matlab called 'SLSendController')
	else
	{
		// if first step then get the gains of the PD controller from the buffer
		if (step == 1)
		{
			total_reward = 0;

			k = episodeTrajectory.stateBuffer[0];
			d = episodeTrajectory.stateBuffer[1];
		}

		// PD error on the position (wrt the ball, the desired position is the center of the beam)
		double ballDist = ballState.x[_X_] - beamState.x[_X_];

		// PD controller with 0 desired velocity
		u = k * ballDist + d * ballState.xd[_X_];
		
		// we control only the last joint, i.e. we rotate the hand of the robot
		for (i = 7; i <= N_DOFS; i ++)
		{
			joint_des_state[i].th = u;
	//		joint_des_state[i].th = joint_state[i].th;
	//		joint_des_state[i].thd = joint_state[i].thd;
		}

		// update cumulative reward
		total_reward += calc_reward_ballonbeam();
	}
}

/* Checks if the episode is over. */
static int isStepOver(int commandIdx, double waitForStart, int step)
{
	int isStepOver = 0;
	int i = 0;
	
	episodeState.commandIdx = commandIdx;
	episodeState.episodeID = episodeTrajectory.episodeID;

	if (commandIdx == 1)
	{
		episodeState.state[0] = ballState.x[_X_];
		episodeState.state[1] = ballState.xd[_X_];	
		return reachedInitState;
	}
	
	// if the time is over or the ball falls (the z-axis of the beam is -2)
	if (servo_time - waitForStart > ending_time || ballState.x[_Z_] < -2.0)
	{
		printf("ServoTime: %f, GotDataTime: %f, steps: %d\n", servo_time, waitForStart, step);

		total_reward += calc_reward_ballonbeam();
		episodeState.state[0] = total_reward;

		for (i = 1; i < N_CART; ++i) 
		{   	             
			episodeState.state[i] = ballState.x[i];
		}
		
		firstGotoPos = 1;
		isStepOver   = 1;
		
		joint_des_state[7].th 	= 0.f;
		joint_des_state[7].thd 	= 0.f;
		joint_des_state[7].thdd = 0.f;
		
		ballState.x[_X_] 	= 0.f;
		ballState.xd[_X_] 	= 0.f;
		ballState.xdd[_X_] 	= 0.f;
		
//		printf("Step is over\n");
	}
		
	return isStepOver;
}


void add_ballonbeam_task( void )
{
	addTask("Ball On A Beam Task", initBallOnBeamTask, 
			runBallOnBeamTask, changeBallOnBeamTask);
	updateDataCollectScript();
}


static int initBallOnBeamTask(void)
{
	int j, i, dim;
	
	// check whether any other task is running
	if (strcmp(current_task_name,NO_TASK) != 0) {
		printf("Task can only be run if no other task is running!\n");
		return FALSE;
	}
	
	start_time = task_servo_time;
	
	setDefaultEndeffector();
	endeff[RIGHT_HAND].x[_Z_] = .2;
	
	// go to a save posture
	bzero((char *)&(init_joint_state[1]),N_DOFS*sizeof(init_joint_state[1]));

	for (i = 1; i <= N_DOFS; i++)
	{
			init_joint_state[i].th = 0;
	}
	init_joint_state[4].th = PI/2;
	init_joint_state[7].th = 0;
	
	// open the communication with Matlab
	createEpisodicSharedMemoryCommunication();

	// useInverseDynamics, pGain, dGain, initPos, gotoPosThreshold
	initMotorUtilities(1, 40, 10, init_joint_state, 0.001);

	// clear the graphics
	sendUserGraphics("clearUserGraphics",0,0);

	if (!go_target_wait_ID(init_joint_state))
		return FALSE;
		
	simBallOnBeamTask();
    
	numTrials = 0;
    
	// ready to go
	int ans = 999;
	while (ans == 999) 
	{
		get_int("Enter 1 to start or anthing else to abort ...",ans,&ans);
	}

  	if (ans != 1)
	{
		return FALSE;
	}
		
	return TRUE;
}


static int runBallOnBeamTask(void)
{
	int simPhysics;
	
	// this line calls (in order): doMotionEpisodeStep, isStepOver, writeStepToSharedMemory
	episodicSharedMemoryWait(&doMotionEpisodeStep, &isStepOver, &writeStepToSharedMemory, &simPhysics, 0, 1);

	if (simPhysics)
	{
		simBallOnBeamTask(); // see 'ballonbeam.c'
	}
	sendBallOnBeamGraphics(); // see 'ballonbeam.c'

	check_range(joint_des_state);
	SL_InvDyn(joint_state,joint_des_state,endeff, &base_state, &base_orient);   

	return TRUE;
}


static int changeBallOnBeamTask(void)
{
    get_double("Enter a new k",k,&k);
	simBallOnBeamTask(TRUE);
	return TRUE;
}


double calc_reward_ballonbeam()
{
	if (ballState.x[_Z_] < -2.0)
	{
		// penalty if the ball falls from the beam
		return - (ending_time - (servo_time - wait_for_start)) * task_servo_rate * sqr(beamLength/2.f);
	}
	// squared distance between the ball and the center of the beam
    return - sqr(ballState.x[_X_] - beamState.x[_X_]);
}
