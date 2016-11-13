#include "SL.h"
#include "SL_man.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"

#include <unistd.h>
#include <math.h>


#include "ext_vision.h"



static ext_vision_comm_s ext_vision_ctx;
static ext_vision_calib_s ext_vision_calib_ctx;

//Can be changed when importing or exporting the matrix!
static char transMFilename[100] = "CameraTransfMatrix.cf";


static int vision_up;
static SL_Cstate vision_obs;
static SL_Cstate vision_tr;



enum calib_state_t {
	noOp = 0,
	collecting_data_still,
	collecting_data_moving,
	disp_tr_data
};

static enum calib_state_t calib_state;


enum robot_state_t {
	gravity_comp = 0,
	follow_ball
};

static enum robot_state_t robot_state;


static void log_init ( void ) {

	int i = 0;
	char varname[30];

	//Vision Obs
	addVarToCollect((char *)&(vision_up), "vision_update", "-", INT, FALSE);
	for ( i = _X_; i <= _Z_; i++ ) {
		sprintf( varname, "vision_obs.x[%d]", i );
		addVarToCollect( (char *) &(vision_obs.x[i]),varname,"m",DOUBLE,FALSE );

		sprintf( varname, "vision_obs.xd[%d]", i );
		addVarToCollect( (char *) &(vision_obs.xd[i]),varname,"m/s",DOUBLE,FALSE );

	}

	//State
	addVarToCollect( (char *) &(calib_state),"calib_state","-",INT,FALSE );
	addVarToCollect( (char *) &(robot_state),"robot_state","-",INT,FALSE );

	updateDataCollectScript();

}




static int get_new_vision_data ( void ) {

	vision_up = 0;

	static double prevTimestamp = 0.0;
	if ( (task_servo_time - prevTimestamp )> 0.020 ) {
		

		if (ext_vision_comm_readNewDataNb ( &ext_vision_ctx, &vision_obs ) ) {
			vision_up = 1;
			prevTimestamp = task_servo_time;
			return 1;
		}

	}
	
	return 0;

}

static void display_vision_data ( SL_Cstate* data ) {


	raw_blobs[1].status = 1;

	int i;
	for(i=_X_; i<=_Z_; i++){
		raw_blobs[1].x[i] = data->x[i];
		//raw_blobs[1].blob.xd[i] = data->xd[i];
		//raw_blobs[1].blob.xdd[i] = 0.0f;

	}
	send_raw_blobs();


}


static int isRobotMoving ( void ) {

	int i, isMoving = 0;
	for ( i = 1; i <= N_DOFS; i++ ) {
		if ( fabs( joint_state[i].thd ) > .1 ) {
			isMoving = 1;
			break;
		}
	}

	return isMoving;

}



static void go_no_op ( void ) {

	calib_state = noOp;
	robot_state = gravity_comp;

}


static void begin_callib_data_col ( void ) {

	if ( calib_state == collecting_data_still	|| calib_state == collecting_data_moving )
		return;


	int whileMoving =  0;

	if ( !get_int( "Collect data while moving ...", whileMoving,&whileMoving ) ) {
		printf( "Error in reading number.\n" );
		return;
	}

	printf("Start collecting calibration data.\n");

	calib_state = whileMoving ? collecting_data_moving : collecting_data_still;

}


static void end_callib_data_col ( void ) {

	if ( calib_state == collecting_data_still	|| calib_state == collecting_data_moving ) {
		calib_state = noOp;
		printf("Stop collecting calibration data.\n");
		usleep( 3.0f / task_servo_rate );
	}

}


static void reset_observations ( void ) {

	int realloc = 0;

	if ( !get_int( "Reallocate matrices ...", realloc,&realloc ) ) {
		printf( "Error in reading number.\n" );
		return;
	}

	int resume = 0;

	if ( calib_state == collecting_data_still || calib_state == collecting_data_moving  ) {
		end_callib_data_col();
		get_int("Want to resume data collection afterwards?",resume,&resume);
	}

	printf("Reseting observations...");
	ext_vision_calib_reset( &ext_vision_calib_ctx, realloc );
	printf("[OK]\n");

	if ( resume )
		begin_callib_data_col();

}


static void import_calibration ( void ) {

	if ( !get_string("Select filename...", transMFilename,transMFilename ) ) {
		printf( "Error in reading filename.\n" );
		return;
	}

	printf("Importing calibration from %s...",transMFilename);
	int ok = ext_vision_calib_importTransMatrix ( &ext_vision_calib_ctx,transMFilename );
	printf("[%s]\n", ok ? "OK" : "FAIL");

}


static void export_calibration ( void ) {

	if ( !get_string("Select filename...", transMFilename,transMFilename ) ) {
		printf( "Error in reading filename.\n" );
		return;
	}

	printf("Exporting calibration in %s...",transMFilename);
	int ok = ext_vision_calib_exportTransMatrix ( &ext_vision_calib_ctx,transMFilename );
	printf("[%s]\n", ok ? "OK" : "FAIL");

}

static void print_calibration ( void ) {

	if ( ext_vision_calib_ctx.isTransfValid )
		ext_vision_calib_printTransMatrix( &ext_vision_calib_ctx );
	else {
		printf("No calibration detacted!\n");
		printf("Calibrate or import calibration first.\n");
	}

}


static void calibration_num ( void ) {

	printf("Current number of calibration points: %ld\n",ext_vision_calib_ctx.observ_size);
	printf("Max  number of calibration points:    %ld\n",ext_vision_calib_ctx.observ_max_size);

}

static void calibration_max ( void ) {


	int newMax = ext_vision_calib_ctx.observ_max_size;
	int keepOld = 0;

	if ( ! get_int("Select maximum number of calibration points ...",newMax,&newMax) ) {
		printf("Error in reading number.\n");
		return;
	}


	if ( ! get_int("Keep old calibration points [0 for no]",keepOld,&keepOld) ) {
		printf("Error in reading number.\n");
		return;
	}


	keepOld = keepOld ? 1 : 0;

	int resume = 0;

	if ( calib_state == collecting_data_still || calib_state == collecting_data_moving  ) {
		end_callib_data_col();
		get_int("Want to resume data collection afterwards?",resume,&resume);
	}

	printf("Setting new max... ");
	ext_vision_calib_setMax( &ext_vision_calib_ctx, newMax, keepOld );
	printf("[OK]\n");

	if ( resume )
		begin_callib_data_col();

}


static void calibration_calc ( void ) {

	int resume = 0;

	if ( calib_state == collecting_data_still || calib_state == collecting_data_moving  ) {
		end_callib_data_col();
		get_int("Want to resume data collection afterwards?",resume,&resume);
	}

	printf("Calculating transformation matrix... ");
	ext_vision_calib_calcTrans( &ext_vision_calib_ctx );
	printf("[OK]\n");

	if ( resume )
		begin_callib_data_col();

}


static void disp_calib ( void ) {

	if ( calib_state == collecting_data_still || calib_state == collecting_data_moving  )
		end_callib_data_col();

	calib_state = disp_tr_data;

}


static void select_robot_mode ( void ) {

	get_int("Select robot_mode: [0-1]",robot_state,(int*)&robot_state);

}







static int init_vision_calib_task ( void ) {


	calib_state = noOp;
	robot_state = gravity_comp;


	vision_up = 0;


	addToMan( "no_op", "go to null state and gravity copens", go_no_op );
	addToMan( "begin_calib", "begin collecting data for calibration", begin_callib_data_col );
	addToMan( "end_calib", "stop collecting data for calibration", end_callib_data_col );
	addToMan( "reset_calib", "reset calibration data", reset_observations );

	addToMan( "import_calib", "import calibration matrix", import_calibration );
	addToMan( "export_calib", "export calibration matrix", export_calibration );
	addToMan( "print_calib", "print calibration data", print_calibration );

	addToMan( "disp_calib_points", "current number of calibration points", calibration_num );
	addToMan( "set_calib_points", "set max number of calibration points", calibration_max );

	addToMan( "calc_calib", "calculate calibration matrix", calibration_calc );

	addToMan( "disp_calib", "display calibrated blob", disp_calib );


	addToMan( "sel_mod", "select robot mode", select_robot_mode );

	log_init ();


	ext_vision_calib_init( &ext_vision_calib_ctx );

	return ext_vision_comm_init( &ext_vision_ctx );
}





static int run_vision_calib_task ( void ) {


	switch ( calib_state ) {

	case noOp:
		break;

	case collecting_data_still: {

		int newData = get_new_vision_data();
		display_vision_data( &vision_obs );

		int isMoving = isRobotMoving();

		if ( !isMoving && newData  )
			ext_vision_calib_addPoint( &ext_vision_calib_ctx, &vision_obs, &(cart_state[1])); //TODO

		break;
	}

	case collecting_data_moving: {

		int newData = get_new_vision_data();
		display_vision_data( &vision_obs );

		if (  newData  )
			ext_vision_calib_addPoint( &ext_vision_calib_ctx, &vision_obs, &(cart_state[1]));

		break;
	}

	case disp_tr_data: {

		get_new_vision_data();

		ext_vision_calib_transPoint(&ext_vision_calib_ctx,&vision_obs,&vision_tr);

		display_vision_data( &vision_tr );

		break;
	}


	}



	switch ( robot_state ) {

	case gravity_comp: {

		int i = 0;
		for ( i = 1; i <= N_DOFS; i++ ) {
			joint_des_state[i].th = joint_state[i].th + joint_state[i].thd/task_servo_rate;
			joint_des_state[i].thd = 0.0;
			joint_des_state[i].thdd = 0.0;
			joint_des_state[i].uff = 0.0;
		}

		SL_InvDyn( NULL, joint_des_state, endeff, &base_state, &base_orient );

		for ( i = 1; i <= N_DOFS; i++ )
			joint_des_state[i].thd = joint_state[i].thd;

		check_range(joint_des_state);

		break;
	}


	case follow_ball: {

		static Matrix myJ = 0;
		static Matrix Jt_J_inv = 0;
		static Matrix Jt_x = 0;
		static Matrix q_des = 0;
		static Matrix x_obs = 0;

		if ( Jt_J_inv == 0 ) {

			myJ = my_matrix(1,3,1,N_DOFS);
			Jt_J_inv = my_matrix(1,N_DOFS,1,N_DOFS);
			Jt_x = my_matrix(1,N_DOFS,1,1);
			q_des = my_matrix(1,N_DOFS,1,1);
			x_obs = my_matrix(1,3,1,1);
		}


		int i,j;
		for ( i = 1; i <= 3; i++ ) {
			for ( j = 1; j <= N_DOFS; j++ ) {
				myJ[i][j] = J[i][j];
			}
		}

		double dist = sqrt(
				sqr(vision_tr.x[_X_] - cart_state[1].x[_X_]) +
				sqr(vision_tr.x[_Y_] - cart_state[1].x[_Y_])+
				sqr(vision_tr.x[_Z_] - cart_state[1].x[_Z_]) );

		const double t_time = 1.3;//3.3333 * dist + 0.1;

		x_obs[_X_][1] = (vision_tr.x[_X_] - cart_state[1].x[_X_])/t_time; //TODO
		x_obs[_Y_][1] = (vision_tr.x[_Y_] - cart_state[1].x[_Y_])/t_time;
		x_obs[_Z_][1] = (vision_tr.x[_Z_] - cart_state[1].x[_Z_])/t_time;


		int ok = mat_mult_transpose_normal(myJ,myJ,Jt_J_inv);

		for ( i = 1; i <= N_DOFS; i++ ) 
			Jt_J_inv[i][i] += 0.0001;

		ok &= my_inv_ludcmp( Jt_J_inv, N_DOFS, Jt_J_inv );		

		ok &= mat_mult_transpose_normal(myJ,x_obs,Jt_x);

		ok &= mat_mult(Jt_J_inv,Jt_x,q_des);

		if ( ! ok ) 
			break;

		static double prev_acc[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		for ( i = 1; i <= N_DOFS; i++ ) {/*
			joint_des_state[i].th = joint_state[i].th + q_des[i][1]/task_servo_rate ;
			joint_des_state[i].thd = q_des[i][1];
			double acc = ( q_des[i][1] - joint_state[i].thd ) * task_servo_rate ;
			joint_des_state[i].thdd = fabs(acc) > 50 ? (acc > 0 ? 50 : -50) : acc;
			joint_des_state[i].uff = 0.0;*/

/*
			joint_des_state[i].th = joint_state[i].th;// + q_des[i][1]/task_servo_rate ;
			joint_des_state[i].thd = joint_state[i].thd;

			double acc = ( q_des[i][1] - joint_state[i].thd ) * task_servo_rate ;
			//printf("ACC: %lf\n",acc);
			acc = fabs(acc) > 80 ? (acc > 0 ? 80 : -80) : acc;
			
			acc = 0.9*prev_acc[i]+0.1*acc;
			acc *= 0.9;
			joint_des_state[i].thdd = acc;
			prev_acc[i] = joint_des_state[i].thdd;
			joint_des_state[i].uff = 0.0;*/

			joint_des_state[i].th = joint_state[i].th;// + q_des[i][1]/task_servo_rate ;
			joint_des_state[i].thd = q_des[i][1];

			double acc = ( q_des[i][1] - joint_state[i].thd ) * task_servo_rate ;
			//printf("ACC: %lf\n",acc);
			acc = fabs(acc) > 80 ? (acc > 0 ? 80 : -80) : acc;
			
			acc = 0.9*prev_acc[i]+0.1*acc;
			acc *= 0.9;
			joint_des_state[i].thdd = 0.0;
			prev_acc[i] = joint_des_state[i].thdd;

			double uff =  0.8 * ( q_des[i][1] - joint_state[i].thd );
printf("UFF %d: %lf\n",i,uff);
			
			joint_des_state[i].uff = fabs(uff) > 15 ? (uff > 0 ? 15 : -15) : uff;

		}

		//check_range(joint_des_state);

		//SL_InvDyn( joint_state, joint_des_state, endeff, &base_state, &base_orient );

		check_range(joint_des_state);

		


		break;
	}


	}

	return TRUE;
}







static int change_vision_calib_task ( void ) {

	return TRUE;
}







void add_vision_calib_task ( void ) {

	addTask( "Vision_Calib_Task", init_vision_calib_task, run_vision_calib_task,
			change_vision_calib_task );

}
