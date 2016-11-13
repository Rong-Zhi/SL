/***********************************************************************
 * This file contains the main functions for the beer pong task, 
 * describes the reward function and the controller (policy).
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
#include "beerpong.h"
#include "ddmp.h"

/* global functions */
void add_beerpong_task(void);

/* local functions */
static int init_beerpong_task(void);
static int run_beerpong_task(void);
static int change_beerpong_task(void);
static double calc_reward(void);

static int n_bases = 30;
static int n_pts   = 2000;
static double ending_time = 12; // maximum duration of a trajectory sent with Matlab

static double actionCosts = 0;
static double bounceNoise = 0;

static SL_DJstate defaultState[N_DOFS+1];
static Matrix fcup;
static Matrix fball;
static Matrix delta;
static int runWithMatlab = FALSE;
static int firsttime;
static int recordData = 0;
static int iter; // counter for the number of trajectory executed
static int pos_pts = 1;

static ddmp_x_s ddmp_x;
// indexing starts from 1, never use ddmp_[0]
static ddmp_s ddmp_[1+1]; // only one DMP in the 4th joint


/* Writes in the shared memory at each time step. The array it writes into
 * is the one named 'state' in the function 'SLGetEpisodeMex'. */
static void writeStepToSharedMemory(int numStepInEpisode)
{
	writeStepToSharedMemoryGeneral(numStepInEpisode);
	int i = 0;
	for (i = 1; i <= N_CART; i++)
	{
		episodeStep.state[numStepInEpisode][i - 1]            = ball_state.x[i];
		episodeStep.state[numStepInEpisode][i - 1 + N_CART]   = ball_state.xd[i];
		episodeStep.state[numStepInEpisode][i - 1 + 2*N_CART] = cup_state.x[i];
	}
}


/* Execute one step of the episode. */
static void doMotionEpisodeStep(int numCommand, double waitForStart, int step)
{
	int i;

	switch (numCommand)
	{
		// command 1: go to init state
		case 1:
		{
			if (step == 1)
			{
				init_beerpong_state();
				cup_state.x[_X_] = episodeTrajectory.stateBuffer[0] + cup_state.x[_X_];
				cup_state.x[_Y_] = episodeTrajectory.stateBuffer[1] + cup_state.x[_Y_];
				actionCosts = episodeTrajectory.stateBuffer[2];
				bounceNoise = episodeTrajectory.stateBuffer[3];
				printf("Start Episode: ActionCost: %f, bounceNoise: %f\n", actionCosts, bounceNoise);

			}
			gotoPosMinJerkSL( init_posture );
		}
		// command 2: follow a trajectory
		case 2:
		{
			// if the episode is over go back to resting posture
			if(step >= episodeTrajectory.trajectorySize)
			{
				gotoPosMinJerkSL(init_posture);			
			}
			else
			{
				doMotion(episodeTrajectory.trajectory, step, episodeTrajectory.trajectorySize);
			}
			break;	
		}
	}
}


/* Checks if the episode is over. */
static int isStepOver(int commandIdx, double waitForStart, int step)
{
	int isStepOver = 0;
	int i = 0;
	double reward;
	static double actionReward = 0;
	
	episodeState.commandIdx = commandIdx;
	episodeState.episodeID = episodeTrajectory.episodeID;

	if (commandIdx == 1)
	{
		pos_pts = 1;
		// write in the buffer the final position and velocity of the ball
		episodeState.state[1] = cup_state.x[_X_];
		episodeState.state[2] = cup_state.x[_Y_];
		actionReward = 0;
		return reachedInitState;
	}

	for (i = 1; i <= N_DOFS; i++) 
	{
		actionReward = actionReward + actionCosts * joint_state[i].u * joint_state[i].u;
	}	

	if((servo_time - waitForStart > ending_time) || (step > (episodeTrajectory.trajectorySize + 1000)))
	{
		reward = calc_reward();
		printf("reward: %f, %d\n", reward, step);
	
		// write in the buffer the total reward of the episode...
		episodeState.state[0] = reward - actionReward;

		for (i = 1; i < N_CART; ++i) 
		{
			// ...and the final position of the ball
			episodeState.state[i] = ball_state.x[i];
		}

		isStepOver = 1;
	}
	
	return isStepOver;
}


static int init_beerpong_task(void)
{
	iter = 1;
	
	int i, j;
	firsttime = TRUE;
	
	// define initial posture
	for (i = 1; i <= N_DOFS; ++i) 
	{
		defaultState[i].th   = 0.;
		defaultState[i].thd  = 0.;
		defaultState[i].thdd = 0.;
	}
	defaultState[2].th = 1.0;
	defaultState[4].th = 2.7;

	setDefaultEndeffector();
	endeff[RIGHT_HAND].x[_Z_] = .08;
	
	// allocate memory
	fcup  = my_matrix(1, n_pts, 1, N_CART);
	fball = my_matrix(1, n_pts, 1, N_CART);
	delta = my_matrix(1, n_pts, 1, N_CART);

	// initialize the DMP
	ddmp_x.tau = 10;
	double dt = 1. / (double)task_servo_rate;
	ddmp_x_init(&ddmp_x, n_bases, "x", dt);
	char varname[30];
	for ( i = 1; i <= 1; i++ ) // only 1 DMP...
	{
		sprintf( varname, "DOF_%i", 4 ); // ...in the 4th joint
		ddmp_init( &ddmp_[i], &ddmp_x, varname );
	}
		
	// read the weights for the basis functions of the DMP from a file
	FILE *stream;
	double weight;
	if ((stream = fopen("w_beerpong.txt","r")) == NULL)
	{
		printf("Can't open %s\n","w_beerpong.txt"); 
		return FALSE;
	}
	for (i = 1; i <= 1; i++) { // only 1 DMP
		ddmp_[i].w[0] = n_bases;

		for (j = 1; j <= n_bases; j++) {
			if ( fscanf(stream, "%lf\n", &weight) == -1 )
				printf("End of file\n");
			else
				ddmp_[i].w[j] = weight; // only 1 DMP
		}
	}

	fclose(stream);
	
	// reset the DMP (i.e. reset phase, position and velocity)
	ddmp_x_reset ( &ddmp_x );
	for ( i = 1; i <= 1; i++ ) // only 1 DMP
		ddmp_reset( &ddmp_[i] );

	// set DMP params
	ddmp_[1].y   = 0;
	ddmp_[1].y0  = 0;
	ddmp_[1].z   = 0;
	ddmp_[1].g   = -.5;
	ddmp_[1].gd  = 0;
	ddmp_[1].gf  = ddmp_[1].g;
	ddmp_[1].a   = -1;

	mat_zero(fcup);
	mat_equal_scalar(1e10, fball);

	if (!go_target_wait_ID(defaultState))
		return FALSE;
	
	// ready to go
	runWithMatlab = FALSE;
	int ans = 999;
	while (ans == 999) 
	{
		get_int(" 1 ... Matlab interface\n 2 ... standard replay\n ... anything else to abort",ans,&ans);
	}

  	if (ans != 1 && ans != 2 && ans != 3)
		return FALSE;
		
	if (ans == 1) // interface with Matlab
	{
		runWithMatlab = TRUE;
		createEpisodicSharedMemoryCommunication();
		initMotorUtilities(1, 40, 10, defaultState, 0.001);
	}

	// clear the graphics
	sendUserGraphics("clearUserGraphics",0,0);

	if (ans == 2) // standard replay
	{
		get_int("Do you want to record data? (1 for yes, anything else for no)",recordData,&recordData);
		if(recordData != 1)
			recordData = 0;
	}		

	init_beerpong_state();
	sim_beerpong_state();
	
	return TRUE;
}


static int run_beerpong_task(void) 
{
	int i, j, k, dim;
	static SL_DJstate joint_increment[N_DOFS+1];
	static int goInit = FALSE; // at the beginning the robot is already in the init position...
	static int playback = TRUE; // ...and ready to playback a trajectory...
	static int playback_ok = FALSE; // ...so this checking variable can be set to FALSE
	static int calc_increment = TRUE; // a flag to calculate (only one time) some small increments to reach the initial position smoothly
	static int simPhysics = FALSE;

	// variables used to calculate the increment (see 'calc_increment')
	static double max_range;
	static double goto_speed = .75;
	static int n_steps;
	static int n_goto_steps;
	
	if(firsttime)
	{
		// these assignments are needed if the same task is performed many times
		firsttime = FALSE;
		goInit = FALSE;
		playback = TRUE;
		playback_ok = FALSE;
		init_beerpong_state();
		sim_beerpong_state();
		if (recordData)
		{
			scd(); // start collecting data
		}
	}
	
	if (runWithMatlab == TRUE)
	{
		episodicSharedMemoryWait(&doMotionEpisodeStep, &isStepOver, &writeStepToSharedMemory, &simPhysics, 0, 1);
	}
	else
	{
		if (playback_ok && goInit) // in this case the robot was reaching the init position and it is ready to playback
		{
			goInit = FALSE;
			playback = TRUE;
			sim_beerpong_state();
			pos_pts = 1;
			mat_zero(fcup);
			mat_equal_scalar(1e10, fball);
		}
		if (playback && (ddmp_x.x < 1e-100 || ball_state.x[_Z_] <= (floor_level-ball_radius))) // if the trajectory is over or the ball is on the floor
		{
			if (recordData) // if you were recording, stop and save collected data
			{
				stopcd();
				saveData();
				recordData = 0;
			}
			printf("reward: %f\n", calc_reward());
			if (iter == 1)
			{
				iter++;
			}
			else
			{
				freeze(); // perform only 2 imitations of the trajectory
			}
			
			ddmp_x_reset ( &ddmp_x );
			ddmp_reset ( &ddmp_[1] );
			
			goInit = TRUE;
			playback = FALSE;
			playback_ok = TRUE;
		}
		
		if (playback) // just replay the learned trajectory
		{
			for (i = 1; i <= N_DOFS; ++i)
			{
				joint_des_state[i].th = defaultState[i].th;
				joint_des_state[i].thd = 0.;
				joint_des_state[i].thdd = 0.;
			}

			ddmp_x_step ( &ddmp_x );
			ddmp_step ( &ddmp_[1] );

			joint_des_state[4].th   = defaultState[4].th + ddmp_[1].y;
			joint_des_state[4].thd  = ddmp_[1].yd;
			joint_des_state[4].thdd = ddmp_[1].ydd;
			joint_des_state[4].uff  = 0.0;
		}
		
		if (goInit) // go back to the init position...
		{
			if (calc_increment) // ...if you still have to, calculate some small steps to reach it smoothly
			{
				calc_increment = FALSE; // do it only once, since the DMP is the same and the robot will end up in the same final position
				max_range = 0;
				for (i = 1; i <= N_DOFS; i++)
				{
					joint_increment[i].th = 0;
				}
				
				// calculate the max distance from a joint to the init position...
				for (i = 1; i <= N_DOFS; i++)
				{
					joint_des_state[i].th = joint_state[i].th;
					if (fabs(defaultState[i].th - joint_des_state[i].th) > max_range) 
					{
						max_range = fabs(defaultState[i].th - joint_des_state[i].th);
					}
				}
				n_steps = 0;
				// ...calculate the number of steps to cover such distance at a desired speed...
				n_goto_steps = max_range / goto_speed * task_servo_rate;
				
				// ...and finally use the number of steps to find the increment of each step
				for (i = 1; i <= N_DOFS; i++)
				{
					joint_increment[i].th = (defaultState[i].th - joint_des_state[i].th) / (double)n_goto_steps;
				}
			}
			
			if (++n_steps < n_goto_steps) // if not yet in the init position...
			{
				for (i = 1; i <= n_dofs; i++)
				{
					joint_des_state[i].th   += joint_increment[i].th; // ...do one step...
					joint_des_state[i].thd   = 0.0;
					joint_des_state[i].thdd  = 0.0;
					joint_des_state[i].uff   = 0.0;
				}
				playback_ok = FALSE; // ...and say that the robot is not ready to playback...
			}
			else
			{
				playback_ok = TRUE; // ...otherwise the robot is ready to playback
			}
		}
		else
		{
			calc_increment = TRUE;
			playback_ok = FALSE;
		}
		
		// finally, if you are doing a playback also simulate the physics
		simPhysics = playback;
		if (!playback)
		{
			init_beerpong_state();
		}
	}
	
	// check whether the trajectory might damage the robot
	check_range(joint_des_state);

	// use a full model-based controller 
	SL_InvDyn(joint_state,joint_des_state,endeff, &base_state, &base_orient);
	
	if (simPhysics)
	{
		sim_beerpong_state(bounceNoise);
	
		if (pos_pts <= n_pts)
		{
			for (dim = _X_; dim <= _Z_; dim++)
			{
				fcup[pos_pts][dim] = cup_state.x[dim];
				fball[pos_pts][dim] = ball_state.x[dim];
			}
		}
		
		pos_pts++;
		if (pos_pts > n_pts+1)
		{
			pos_pts = n_pts+1;
		}
	}
	send_beerpong_graphics();
	return TRUE;
}


static int change_beerpong_task(void)
{	
	int i;
	double temp;

	// change the initial joint position
	for ( i = 1; i <= N_DOFS; i++ )
	{
		printf( "DOF %i", i );
		get_double( ":", defaultState[i].th, &temp );
		defaultState[i].th = temp;
	}

	check_range( defaultState );

	return TRUE;
}


void add_beerpong_task( void )
{
	addTask("Beer Pong Task", init_beerpong_task, 
			run_beerpong_task, change_beerpong_task);			
	
	add_beerpong_vars();
	
	updateDataCollectScript();	
}    


static double calc_reward(void)
{
	int i;
	
	mat_sub(fcup, fball, delta);
	
	double reward_top = 1e10;
	double reward_middle = 1e10;
	double reward_bottom = 1e10;
	double reward_temp;
	
	for (i = 1; i <= n_pts; i++)
	{
		reward_temp = sqr(delta[i][1]) + sqr(delta[i][2]) + sqr(delta[i][3]);
		if (reward_temp < reward_bottom)
		{
			reward_bottom = reward_temp;
		}
		reward_temp = sqr(delta[i][1]) + sqr(delta[i][2]) + sqr(delta[i][3] + .5 * cup_height);
		if (reward_temp < reward_middle)
		{
			reward_middle = reward_temp;
		}
		reward_temp = sqr(delta[i][1]) + sqr(delta[i][2]) + sqr(delta[i][3] + cup_height);
		if (reward_temp < reward_top)
		{
			reward_top = reward_temp;
		}
	}
	return -(reward_bottom + reward_middle + reward_top) * 100.;
}
