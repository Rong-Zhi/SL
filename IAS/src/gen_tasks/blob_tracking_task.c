#include "SL_system_headers.h"
#include "SL.h"
#include "SL_man.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"

/* Internal Variables */
static double learning_rate = 0.05;
static double start_time; 
static int endEff_idx;
static int blob_idx;

static Vector x_err;
static Vector q_des;
static Vector qd_des;
static Vector qdd_ref;
static Matrix J_trans;
static Matrix myJ;


static int init_blob_tracking_task(void) {
	
	endEff_idx = 0;
	while(! (endEff_idx >= 1 && endEff_idx <= N_ROBOT_ENDEFFECTORS) ) {
		get_int("Insert end effector index",endEff_idx,&endEff_idx);
	}

	blob_idx = 0;
	while(! (blob_idx >= 1 && blob_idx <= max_blobs) )
		get_int("Insert blob index",blob_idx,&blob_idx);
	
	// Memory allocations go here
	x_err	= my_vector(1,N_CART);
	q_des	= my_vector(1,N_DOFS);
	qd_des	= my_vector(1,N_DOFS);
	qdd_ref = my_vector(1,N_DOFS);
	J_trans	= my_matrix(1,N_DOFS,1,N_CART);
	myJ		= my_matrix(1,N_CART,1,N_DOFS);

	start_time = servo_time;
	
	return TRUE;
}


static int run_blob_tracking_task(void) {
	double t, period;
	int i, j;
	double damping = .1;
	
	// Computes current time t
	t = servo_time - start_time;

	// Extract only the needed rows from the Jacobian
	for(i = 1; i <= N_CART; i++) {
		for(j = 1; j <= N_DOFS; j++) {
			myJ[i][j] = J[i][j];
		}
	}
	
	// Compute the Jacobian transpose
	int ok = mat_trans(myJ, J_trans);

	// Compute the error in the task space
	SL_Cstate* vision_data = &(blobs[blob_idx].blob);
	x_err[1] = (vision_data->x[_X_] - cart_state[endEff_idx].x[_X_]) * learning_rate * task_servo_rate;
	x_err[2] = (vision_data->x[_Y_] - cart_state[endEff_idx].x[_Y_]) * learning_rate * task_servo_rate;
	x_err[3] = (vision_data->x[_Z_] - cart_state[endEff_idx].x[_Z_]) * learning_rate * task_servo_rate;

	// Use the Jacobian transpose for inverse kinematics
	ok &= mat_vec_mult(J_trans,x_err,qd_des);

	if ( ! ok )
		return FALSE;

	// Integrate the velocity to obtain the position
	for (i = 1; i <= N_DOFS; ++i) {
		q_des[i] = qd_des[i] / task_servo_rate;
	}

	// Gains
	float K_p[7] = {200., 300., 100., 50., 20., 20., 2.5}; // change according to the robot
	float K_d[7] = {7., 15., 5., 2.5, 2.5, 0.5, 0.075}; // change according to the robot

	/* PD CONTROLLER */
	for (i = 1; i <= N_DOFS; ++i) {
		joint_des_state[i].th   = joint_state[i].th + q_des[i];
		joint_des_state[i].thd  = qd_des[i];
		joint_des_state[i].thdd = K_d[i-1] * (joint_des_state[i].thd - joint_state[i].thd) + K_p[i-1] * (joint_des_state[i].th - joint_state[i].th);
	}
	
	joint_des_state[i].thd += damping * (joint_des_state[i].thd - joint_state[i].thd);


	/* PD CONTROLLER WITH GRAVITY COMPENSATION */
	//~ SL_InvDyn(NULL, joint_des_state, endeff, &base_state, &base_orient); // it ignores joint_des_state[i].thdd, so in the loop above you could set joint_des_state[i].thdd = 0

	/* MODEL-BASED CONTROLLER */
	SL_InvDyn(joint_state, joint_des_state, endeff, &base_state, &base_orient);
	
	// Check whether the trajectory might damage the robot
	check_range(joint_des_state);
	
	return TRUE;
}


static int change_blob_tracking_task(void) {
	return TRUE;
}


void add_blob_tracking_task ( void ) {
	addTask( "Blob_Tracking_Task", init_blob_tracking_task, run_blob_tracking_task,
			change_blob_tracking_task );
}
