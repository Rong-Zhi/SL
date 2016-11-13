/* global includes */
#include "SL_system_headers.h"
#include "SL.h"
#include "SL_user.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_dynamics.h"
#include "SL_tasks.h"

#define endEff_idx 1

/* internal variables */
static double learning_rate = 0.05;
static int simulate_a_ball = 1;
static double start_time; 
static double ball_speed = 1.0;

static Vector error, derror;
static Vector x_err;
static Vector q_des;
static Vector qd_des;
static Vector qdd_ref;
static Matrix J_trans;
static Matrix subJ;

/* global functions */
void add_ballonbeam_task(void);

/* local functions */
static int init_ball_following_task(void);
static int run_ball_following_task(void);
static int change_ball_following_task(void);


/*******************************************************
* Produces a moving ball
*******************************************************/
static int simulate_ball(void) {
	int j, i;
	static int last_frame_counter = -999;
	static int sendflag = FALSE;

	// Ball Simulator
	raw_blobs[BLOB1].status = TRUE;
	raw_blobs[BLOB1].x[_X_] = 0.25*sin(2.*PI*0.25*ball_speed*(task_servo_time-start_time));
	raw_blobs[BLOB1].x[_Y_] = -0.75;
	raw_blobs[BLOB1].x[_Z_] = -0.8;
	
	// Simulated Vision Servo
	if (sendflag) {
		send_raw_blobs();
		sendflag = FALSE;
	}
	if (last_frame_counter != frame_counter) {
		sendflag = TRUE;
		last_frame_counter = frame_counter;
	}
	
	return TRUE;
}

/********************************************************
* Reads the ball position and send it to the vision servo
********************************************************/
static void show_two_balls() {
    double pos[2*N_CART+1];
	
    pos[_X_]	 = raw_blobs[BLOB1].x[_X_];
    pos[_Y_] 	 = raw_blobs[BLOB1].x[_Y_];
    pos[_Z_] 	 = raw_blobs[BLOB1].x[_Z_];
    pos[3 + _X_] = blobs[1].blob.x[_X_]; 
    pos[3 + _Y_] = blobs[1].blob.x[_Y_];
    pos[3 + _Z_] = blobs[1].blob.x[_Z_];
	
    sendUserGraphics("twoballs",&(pos[_X_]), 2*N_CART*sizeof(double));
}

/********************************************************
* Initialization (this function is called only once).
********************************************************/
static int init_ball_following_task(void) {
	int ans, i, j;
	char string[100];	
	
	error  = my_vector(1,N_CART);
	derror = my_vector(1,N_DOFS);		

	// Add variables for MRDPLOT
	for (i = _X_; i <= _Z_; ++i) {
		sprintf(string,"error_%d",i);
		addVarToCollect((char *)&(error[i]),string,"m", DOUBLE,FALSE);
	}
	for (i = 1; i <= N_DOFS; ++i) {
		sprintf(string,"derror_%d",i);
		addVarToCollect((char *)&(derror[i]),string,"rad", DOUBLE,FALSE);
	}
	updateDataCollectScript();	

	// Anything done once on static variables and all memory allocations go here
	x_err	= my_vector(1,N_CART);
	q_des	= my_vector(1,N_DOFS);
	qd_des	= my_vector(1,N_DOFS);
	qdd_ref = my_vector(1,N_DOFS);
	J_trans	= my_matrix(1,N_DOFS,1,N_CART);
	subJ	= my_matrix(1,N_CART,1,N_DOFS);

	start_time = servo_time;

	// Ask the user if he wants to simulate also the ball and, if so, at which speed
	get_int("Simulate ball?", simulate_a_ball, &simulate_a_ball);
	if(simulate_a_ball)
		get_double("Simulated ball speed",ball_speed,&ball_speed);
	
	// The user can now start the task or cancel it if he changed is mind
	ans = 999;
	while (ans == 999) {
		if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
			return FALSE;
	}
	
	printf("Starting ...\n");
	
	scd();
	
	return TRUE;
}

/********************************************************
* The core function. It is called at each timestep.
********************************************************/
static int run_ball_following_task(void) {
	double t, period;
	int i, j;
	
	if(simulate_a_ball)
		simulate_ball();
	show_two_balls();
	
	// Computes current time t
	t = servo_time - start_time;

	// Extract only the needed rows from the Jacobian
	for(i = 1; i <= N_CART; i++) {
		for(j = 1; j <= N_DOFS; j++) {
			subJ[i][j] = J[i][j];
		}
	}
	
	// Compute the Jacobian transpose
	mat_trans(subJ, J_trans);
	
	// Compute the error in the task space
	x_err[1] = (raw_blobs[BLOB1].x[_X_] - cart_state[endEff_idx].x[_X_]) * learning_rate * task_servo_rate; // task_servo_rate is a global variable
	x_err[2] = (raw_blobs[BLOB1].x[_Y_] - cart_state[endEff_idx].x[_Y_]) * learning_rate * task_servo_rate;
	x_err[3] = (raw_blobs[BLOB1].x[_Z_] - cart_state[endEff_idx].x[_Z_]) * learning_rate * task_servo_rate;

	// Use the Jacobian transpose for inverse kinematics
	mat_vec_mult(J_trans,x_err,qd_des);

	// Integrate the velocity to obtain the position
	for (i = 1; i <= N_DOFS; ++i) {
		q_des[i] = qd_des[i] / task_servo_rate;
	}

	// Gains
	float K_p[7] = {200., 300., 100., 50., 20., 20., 2.5}; // these gains are copied from the slides
	float K_d[7] = {7., 15., 5., 2.5, 2.5, 0.5, 0.075}; // you can also use adaptive gains that increase slowly until a max value

	/* PD CONTROLLER */
	double damping = 5;
	double dt = 1.0 / task_servo_rate;
	for (i = 1; i <= N_DOFS; ++i) {
		joint_des_state[i].th   = joint_state[i].th + q_des[i];
		joint_des_state[i].thd  = qd_des[i];
		joint_des_state[i].thdd = K_d[i-1] * (joint_des_state[i].thd - joint_state[i].thd) + K_p[i-1] * (joint_des_state[i].th - joint_state[i].th);
		joint_des_state[i].th  += damping * dt * (joint_des_state[i].thd - joint_state[i].thd);
	}

	// Check whether the trajectory might damage the robot
	//check_range(joint_des_state);
	
	/* FEEDFORWARD CONTROLLER */
	//SL_InvDyn(NULL, joint_des_state, endeff, &base_state, &base_orient);

	/* MODEL-BASED CONTROLLER */
	SL_InvDyn(joint_state, joint_des_state, endeff, &base_state, &base_orient);
	
	return TRUE;
}

/********************************************************
* Default function.
********************************************************/
static int change_ball_following_task(void) {
	return TRUE;
}

/********************************************************
* Default function. Adds the task to the task menu.
********************************************************/
void add_ball_following_task( void ) {
	addTask("Ball Following Task", init_ball_following_task, 
			run_ball_following_task, change_ball_following_task);			
}  
