#include "SL.h"
#include "SL_man.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "SL_task_servo.h"

#include "ias_utilities.h"
#include "ias_matrix_utilities.h"
#include "trajectories.h"



enum run_state_t {
	noOp = 0,
	splineInit,
	splineExec,
	splinePost,
	waiting
};

static enum run_state_t run_state;



static Matrix poses = 0;
static int curPoseIdx = 0;
static double ttw = 2.0;



static void read_from_file () {

	if ( poses!= 0 )
		free(poses); //TODO myFree?

	static char filename[100] = "spline.poses";

	if ( !get_string("Select filename...", filename,filename ) ) {
		printf( "Error in reading filename.\n" );
		return;
	}

	printf("Importing poses from %s...",filename);
	int nPose = fread_mat_ascii(filename,&poses,2*N_DOFS+2);
	int ok =  nPose <= 0 ? 0 : 1;
	printf("[%s]\n", ok ? "OK" : "FAIL");
	if ( ok )
		printf("Imported %d poses\n",nPose);

}



static void set_delay_between_poses ( void ) {

	printf("Set delay value in seconds\n");
	printf("Zero for continuous movement\n");
	printf("Negative value for manual op\n");
	if ( !  get_double("Set delay between poses in sec:",ttw,&ttw) )
		printf( "Error in reading delay.\n" );

}


static void goto_pose ( void ) {

	if ( poses == 0 || run_state == splineInit || run_state == splineExec ) {
		printf("Can't set pose while executing motion, aborting...\n");
		return;
	}

	int aux = -1;
	get_int("Set goto pose:",curPoseIdx,&aux);
	if ( aux >=1 && aux <= (int)poses[0][NR] ) {
		curPoseIdx = aux;
		run_state = splineInit;
	}
	else
		printf("Invalid pose aborting...\n");


}



static int init_spline_task ( void ) {

	static int init = 0;
	if ( ! init ) {
		init = 1;

		addToMan( "set_delay", "set delay between the poses", set_delay_between_poses );
		addToMan( "goto_pose", "start a specific pose in the sequence", goto_pose );

		addToMan( "import_poses", "import poses from file", read_from_file );

	}

	read_from_file();

	run_state = noOp;

	return TRUE;
}




static int run_spline_task ( void ) {

	static SL_DJstate joint_goal_state[N_DOFS+1];
	static SL_DJstate joint_start_state[N_DOFS+1];
	static SL_DJstate joint_via_state[N_DOFS+1];

	static int hasVia = 0;
	static double trajTime = 0.0;
	static double trajViaTime = 0.0;
	static double curTime = 0.0;

	static int w_steps;
	static int w_c_step;


	switch (run_state) {

	case noOp:
		break;


	case splineInit: {

		if ( poses == 0 || curPoseIdx < 1 || curPoseIdx > poses[0][NR]  ) {
			run_state = noOp;
			break;
		}

		run_state = splineExec;

		hasVia = 0;

		if ( curPoseIdx+1 <= poses[0][NR] ) { // check for via point
			if ( poses[curPoseIdx][2*n_dofs+2] > 0.1 ) { // has via point
				hasVia = 1;
				curPoseIdx += 1;  // store to goal_state the next pose
			}
		}



		int i;
		for (i=1; i<=n_dofs; ++i) {
			joint_goal_state[i].th = poses[curPoseIdx][i];
			joint_goal_state[i].thd = poses[curPoseIdx][n_dofs+i];
			if ( hasVia ) {
				joint_via_state[i].th = poses[curPoseIdx-1][i];
				joint_via_state[i].thd = poses[curPoseIdx-1][n_dofs+i];
			}
		}

		check_range(joint_goal_state);

		if ( hasVia )
			check_range(joint_via_state);


		trajTime = poses[curPoseIdx][2*n_dofs+1];
		if ( hasVia )
			trajViaTime = poses[curPoseIdx-1][2*n_dofs+1];

		curTime = 0.0f;

		for (i=1; i<=n_dofs; ++i) {
			joint_start_state[i].th = joint_state[i].th;
			joint_start_state[i].thd = joint_state[i].thd;
			joint_start_state[i].thdd = joint_state[i].thdd;
		}

		if ( hasVia )
			s5via(joint_des_state, joint_start_state, joint_goal_state, joint_via_state, curTime, trajViaTime, trajTime, 1);

	}

	case splineExec:
	{
		if ( ! hasVia ) {
			spline5th(joint_des_state, joint_start_state, joint_goal_state, curTime, trajTime);
		}
		else {
			s5via(joint_des_state, joint_start_state, joint_goal_state, joint_via_state, curTime, trajViaTime, trajTime, 0);
		}

		curTime += 1.0 / task_servo_rate;

		SL_InvDyn( 0, joint_des_state, endeff, &base_state, &base_orient );

		joint_des_state[1].th = 0.0;
		joint_des_state[5].th = 0.0;

		check_range(joint_des_state);

		if ( curTime >= trajTime)
			run_state = splinePost;

//		printf("curTime %f TrajTime %f Inc %f\n",curTime,trajTime,1.0 / task_servo_rate);

		break;
	}


	case splinePost: {

		if ( ttw < 0 ) {
			run_state = noOp;
			break;
		}

		if ( poses!= 0 && curPoseIdx < poses[0][NR]  )
			curPoseIdx += 1;
		else {
			run_state = noOp;
			break;
		}

		if ( ttw < 0.001 )
			run_state = splineInit;
		else {
			run_state = waiting;
			w_c_step = 0;
			w_steps = ttw * task_servo_rate -1;
		}

		break;
	}


	case waiting: {

		if (++w_c_step >= w_steps)
			run_state = splineInit;

		break;
	}


	}



	  return TRUE;

}


static int change_spline_task ( void ) {

	return TRUE;
}



void add_spline_task ( void ) {

	addTask( "spline_task", init_spline_task, run_spline_task,
			change_spline_task );

}
