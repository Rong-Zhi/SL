#include "SL.h"
#include "SL_man.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"

#include <math.h>
#include "ias_utilities.h"
#include "ias_matrix_utilities.h"
#include "ias_motor_utilities.h"



static Matrix poses = 0;
static int curPoseIdx = 0;
static double ttw = 2.0;

enum goto_state_t {
	noOp = 0,
	gravComp,
	goingToInit,
	goingTo,
	waitingInit,
	waiting
};

static enum goto_state_t goto_state;


static double goto_speed = 1.0;




static void read_from_file () {

	if ( poses!= 0 )
		free(poses); //TODO myFree?

	static char filename[100] = "Goto.poses";

	if ( !get_string("Select filename...", filename,filename ) ) {
		printf( "Error in reading filename.\n" );
		return;
	}

	printf("Importing poses from %s...",filename);
	int nPose = fread_mat_ascii(filename,&poses,N_DOFS);
	int ok =  nPose <= 0 ? 0 : 1;
	printf("[%s]\n", ok ? "OK" : "FAIL");
	if ( ok )
		printf("Imported %d poses\n",nPose);	

}


static void write_to_file () {

	if ( poses== 0 ) {
		printf("Poses are empty!\n Aborting...\n");
		return;
	}

	static char filename[100] = "Goto.poses";

	if ( !get_string("Select filename...", filename,filename ) ) {
		printf( "Error in reading filename.\n" );
		return;
	}

	printf("Exporting poses to %s...",filename);
	int ok = fwrite_mat_ascii(filename,poses);
	printf("[%s]\n", ok ? "OK" : "FAIL");

}




static void set_goto_speed ( void ) {

	double aux = 1.0;
    if ( ! get_double("Set goto speed:",goto_speed,&aux) ) {
    	printf( "Error in reading speed.\n" );
    	return;
    }
    if ( aux <= 0.0 || aux > 3.0  )
    	printf( "Desired speed out of limits %lf.\n", aux );
    else
    	goto_speed = aux;

}


static void set_delay_between_poses ( void ) {

	printf("Set delay value in seconds\n");
	printf("Zero for continuous movement\n");
	printf("Negative value for manual op\n");
	if ( !  get_double("Set delay between poses in sec:",ttw,&ttw) )
		printf( "Error in reading delay.\n" );

}


static void goto_next_pose ( void ) {

	if ( poses != 0 && goto_state != goingToInit && goto_state != goingTo ) {
		curPoseIdx = (curPoseIdx % (int)poses[0][NR]) +1;
		goto_state = goingToInit;
	}

}


static void goto_pose ( void ) {

	if ( poses == 0 || goto_state == goingToInit || goto_state == goingTo ) {
		printf("Can't set pose while executing motion, aborting...\n");
		return;
	}

	int aux = -1;
	get_int("Set goto pose:",curPoseIdx,&aux);
	if ( aux >=1 && aux <= (int)poses[0][NR] ) {
		curPoseIdx = aux;
		goto_state = goingToInit;
	}
	else
		printf("Invalid pose aborting...\n");


}


static void save_pos ( void ) {

	int aux;
	get_int("Set goto pose:",curPoseIdx,&aux);


}

static void set_goto_pos ( void ) {

	if ( goto_state == goingToInit || goto_state == goingTo ) {
			printf("Can't set pose while executing motion, aborting...\n");
			return;
	}

	//append poses...

//	for (i=1; i<=N_DOFS; ++i) {
//
//		sprintf(string,"%s: target theta",joint_names[i]);
//		joint_goto_state[i].th = joint_des_state[i].th;
//		while (TRUE)  {
//			if (!get_double(string,joint_goto_state[i].th,&joint_goto_state[i].th))
//				return FALSE;
//			if (joint_goto_state[i].th > joint_range[i][MAX_THETA] ||
//					joint_goto_state[i].th < joint_range[i][MIN_THETA]) {
//				printf("Joint limits are from %5.3f to %5.3f\n",
//						joint_range[i][MIN_THETA],
//						joint_range[i][MAX_THETA]);
//				joint_goto_state[i].th = joint_des_state[i].th;
//				beep(1);
//			} else {
//				break;
//			}
//		}
//	}
}


static void toggle_grav_comp ( void ) {

	goto_state = goto_state==gravComp ? noOp : gravComp;

}





static int get_theta_in_range ( int i, double* res, int isMin ) {

	char strBuf[100];

	sprintf( strBuf, "%s: %s theta", joint_names[i], isMin? "min" : "max" );
	while ( TRUE ) {
		if ( !get_double( strBuf, joint_state[i].th, res ) ) {
			printf("Problem reading theta, aborting...\n");
			return FALSE;
		}
		if ( *res > joint_range[i][MAX_THETA] || *res < joint_range[i][MIN_THETA] ) {
			printf( "Joint limits are from %5.3f to %5.3f\n",
					joint_range[i][MIN_THETA], joint_range[i][MAX_THETA] );
		}
		else
			break;
	}

	return TRUE;

}


static int get_step ( int i, double* res ) {

	char strBuf[100];

	sprintf( strBuf, "%s: step theta", joint_names[i] );

	if ( !get_double( strBuf, 0.2, res ) ) {
		printf( "Problem reading step, aborting...\n" );
		return FALSE;
	}

	if ( *res < 0.005 ) {
		printf("Step size too low, setting to 0.2\n");
		*res = 0.2;
	}

	return TRUE;

}


static void gen_pos_rec ( int* pos_idx, int dof_idx, double* min, double* max, double* step, double* cur) {

	if ( dof_idx > N_DOFS ) {
		int i;

		for ( i = 1; i <= N_DOFS; ++i )
			poses[*pos_idx][i] = cur[i];

		*pos_idx += 1;

		return;
	}

	int steps = floor(fabs(max[dof_idx]-min[dof_idx]) / fabs(step[dof_idx]))+1;
	int si = max[dof_idx]-min[dof_idx] > 0 ? 1 : -1;

	int i;
	cur[dof_idx] = min[dof_idx];
	for ( i = 1; i <= steps; ++i ) {
		gen_pos_rec ( pos_idx,dof_idx+1,min,max,step,cur);
		cur[dof_idx] = min[dof_idx] + si*i*fabs(step[dof_idx]);
	}

}



static void generate_grid_pos ( void )  {

	double 	min_angle[N_DOFS+1], max_angle[N_DOFS+1], step[N_DOFS+1];

	int ok = 1;
	int tPoses = 1;

	int i;
	for ( i = 1; i <= N_DOFS; ++i ) {

		ok &= get_theta_in_range(i,	&(min_angle[i]), 1);
		ok &= get_theta_in_range(i,	&(max_angle[i]), 0);

		ok &= get_step(i, &(step[i]) );

		tPoses *= floor(fabs(max_angle[i]-min_angle[i]) / fabs(step[i])) + 1;

	}

	if ( ! ok ) {
		printf("Problem reading values, aborting.\n");
		return;
	}

	//TODO append to matrix

	if ( poses != 0 )
		free(poses); //TODO myfree?



	poses = my_matrix(1,tPoses,1,N_DOFS);

	double curPos[N_DOFS+1];
	int temp = 1;

	gen_pos_rec(&temp,1,min_angle,max_angle,step,curPos);

	print_mat("asdf",poses);




}









static int init_grid_goto_task ( void ) {

	static int init = 0;

	if ( ! init ) {
		init = 1;

		double aux;
		if (read_parameter_pool_double(config_files[PARAMETERPOOL],"goto_speed",&aux)) {
			if (aux > 0 && aux < 3.0)
				goto_speed = aux;
			else
				printf("Invalid goto_speed (%f)in parameter pool file\n",aux);
		}

		addToMan( "set_goto_speed", "set the goto speed", set_goto_speed );
		addToMan( "set_delay", "set delay between the poses", set_delay_between_poses );
		addToMan( "goto_next", "start the next pose in the sequence", goto_next_pose );
		addToMan( "goto_pose", "start a specific pose in the sequence", goto_pose );

		addToMan( "import_poses", "import poses from file", read_from_file );
		addToMan( "export_poses", "export poses to file", write_to_file );

		addToMan( "gravity_comp", "Toggle gravity compensation", toggle_grav_comp );


		addToMan( "generate_grid_pos", "Toggle gravity compensation", generate_grid_pos );
	}

	goto_state = noOp;

	read_from_file();

	return TRUE;

}


static int run_grid_goto_task ( void ) {

	static SL_DJstate joint_goto_state[N_DOFS+1];
	static int n_steps;
	static int n_goto_steps;

	static int w_steps;
	static int w_c_step;


	switch (goto_state) {

	case noOp: {
		int i;
		for (i=1; i<=n_dofs; ++i) {
			joint_des_state[i].th = joint_state[i].th;
			joint_des_state[i].thd = 0.0;
			joint_des_state[i].thdd = 0.0;
			joint_des_state[i].uff = 0.0;
		}
		SL_InvDyn( 0, joint_des_state, endeff, &base_state, &base_orient );

		break;
	}

	case gravComp:
	{
		int i = 0;
		for ( i = 1; i <= N_DOFS; i++ ) {
			joint_des_state[i].th = joint_state[i].th + joint_state[i].thd / task_servo_rate;
			joint_des_state[i].thd = 0.0;
			joint_des_state[i].thdd = 0.0;
			joint_des_state[i].uff = 0.0;
		}

		SL_InvDyn( NULL, joint_des_state, endeff, &base_state, &base_orient );

		for ( i = 1; i <= N_DOFS; i++ ) {
			joint_des_state[i].thd = joint_state[i].thd;
			joint_des_state[i].thdd = joint_state[i].thdd;
		}

		check_range( joint_des_state );

		break;
	}


	case goingToInit: {

		if ( poses == 0 || curPoseIdx < 1 || curPoseIdx > poses[0][NR]  ) {
			goto_state = noOp;
			break;
		}

		goto_state = goingTo;

		double max_range=0;

		int i;
		for (i=1; i<=n_dofs; ++i) {
			joint_goto_state[i].th = poses[curPoseIdx][i];
			double range = fabs(joint_goto_state[i].th - joint_state[i].th);
			if (range > max_range)
				max_range = range;
		}
		check_range(joint_goto_state);

		/* ensure that the goal state has zero vel and zero acc */
		for (i=1; i<=n_dofs; ++i)
			joint_goto_state[i].thd = joint_goto_state[i].thdd = 0.0;


		n_steps = 0;
		n_goto_steps = max_range/goto_speed*task_servo_rate;
		if (n_goto_steps == 0) {
			goto_state = noOp;
			break;
		}

		for (i=1; i<=n_dofs; ++i) {
			joint_des_state[i].th = joint_state[i].th;
			joint_des_state[i].thd = joint_state[i].thd;
			joint_des_state[i].thdd = joint_state[i].thdd;
		}

	}

	case goingTo:
	{

		double time_to_go = (n_goto_steps - n_steps)/((double) task_servo_rate);		

		// kinematics follow min jerk
		if (!ias_calculate_min_jerk_next_step (joint_des_state, joint_goto_state,time_to_go)) {
			goto_state = waitingInit;
			printf("Calculate_min_jerk_next_step failed!\n");
			printf("TimeToGo: %lf %d\n",time_to_go,(n_goto_steps - n_steps));
			break;
		}

		SL_InvDyn( joint_state, joint_des_state, endeff, &base_state, &base_orient );

		check_range(joint_des_state);

		if (++n_steps >= n_goto_steps-1) 
			goto_state = waitingInit;

		break;
	}


	case waitingInit: {

		if ( ttw < 0 ) {
			goto_state = noOp;
			break;
		}

		if ( poses!= 0 && curPoseIdx < poses[0][NR]  )
			curPoseIdx += 1;
		else {
			goto_state = noOp;
			break;
		}

		if ( ttw < 0.001 )
			goto_state = goingToInit;
		else {
			goto_state = waiting;
			w_c_step = 0;
			w_steps = ttw * task_servo_rate -1;
		}

		break;
	}


	case waiting: {

		int i;
		for (i=1; i<=n_dofs; ++i) {
			joint_des_state[i].th = joint_state[i].th;
			joint_des_state[i].thd = 0.0;
			joint_des_state[i].thdd = 0.0;
			joint_des_state[i].uff = 0.0;
		}
		SL_InvDyn( 0, joint_des_state, endeff, &base_state, &base_orient );

		if (++w_c_step >= w_steps)
			goto_state = goingToInit;

		break;
	}


	}



	  return TRUE;

}


static int change_grid_goto_task ( void ) {

	return TRUE;

}



void add_grid_goto_task ( void ) {

	addTask( "Grid_Goto_Task", init_grid_goto_task, run_grid_goto_task,
	         change_grid_goto_task );
}

