#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "SL_task_servo.h"


static int init_grav_comp ( void ) {
	return TRUE;
}


static int run_grav_comp ( void ) {

	int i = 0;
	for ( i = 1; i <= N_DOFS; i++ ) {
		joint_des_state[i].th = joint_state[i].th + (joint_state[i].thd + joint_state[i].thdd / task_servo_rate) / task_servo_rate ;
		joint_des_state[i].thd = 0.0;
		joint_des_state[i].thdd = 0.0;
		joint_des_state[i].uff = 0.0;
	}

	//SL_InvDyn( NULL, joint_des_state, endeff, &base_state, &base_orient );

	for ( i = 1; i <= N_DOFS; i++ ) {
		joint_des_state[i].thd = joint_state[i].thd + 0.9 * ( joint_state[i].thd - misc_sim_sensor[MOTOR_ENCODER_R_SFE_D+(i-1)] ) ;
	}

	check_range( joint_des_state );

	return TRUE;
}


static int change_grav_comp ( void ) {

	return TRUE;
}



void add_grav_comp_task ( void ) {

	addTask( "grav_comp", init_grav_comp, run_grav_comp,
			change_grav_comp );

}
