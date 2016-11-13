#include "SL.h"
#include "SL_man.h"
//#include "SL_user.h"
//#include "SL_user_common.h"
#include "SL_vision_servo.h"
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
	collecting_data_grid
};

static enum calib_state_t calib_state;


static double grid_ttw_stab = 0.5;
static double grid_ttgather = 1.0;







static void calibration_calc ( void );



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

		sprintf( varname, "vision_tr.x[%d]", i );
		addVarToCollect( (char *) &(vision_tr.x[i]),varname,"m",DOUBLE,FALSE );

		sprintf( varname, "my_cart.x[%d]", i );
		addVarToCollect( (char *) &(cart_state[1].x[i]),varname,"m",DOUBLE,FALSE );

	}

	//State
	addVarToCollect( (char *) &(calib_state),"calib_state","-",INT,FALSE );

	updateDataCollectScript();

}




static int get_new_vision_data ( void ) {

	vision_up = 0;

	if (ext_vision_comm_readNewDataBlk ( &ext_vision_ctx, &vision_obs ) ) {
		vision_up = 1;
		return 1;
	}
	
	return 0;

}



static int isRobotMoving ( void ) {

	int i, isMoving = 0;
	for ( i = 1; i <= n_dofs; i++ ) {
		if ( fabs( joint_state[i].thd ) > .1 ) {
			isMoving = 1;
			break;
		}
	}

	return isMoving;

}


static int isCollectingData ( void ) {

	if ( calib_state == collecting_data_still
			|| calib_state == collecting_data_moving
			|| calib_state == collecting_data_grid )
			return TRUE;

	return FALSE;
}



static void go_no_op ( void ) {

	calib_state = noOp;

}


static int intReactivTrans = 0; //Shared between begin/end data collection functions

static void begin_callib_data_col ( void ) {

	if ( isCollectingData() )
		return;


	int mode =  1;

	printf( "Select calibration mode: \n" );
	printf( "1. Collect data while moving\n" );
	printf( "2. Collect data when stable\n" );

	if ( !get_int( "Select calibration mode ...", mode,&mode ) ) {
		printf( "Error in reading number.\n" );
		return;
	}

	if ( mode < 1 || mode > 2) {
		printf( "Not supporting mode.\n" );
		return;
	}

	if ( mode == 2 ) {

		int ok = get_double( "Time to wait for robot to stabilize: ", grid_ttw_stab,&grid_ttw_stab );
		ok &= get_double( "Data gathering time for each pose (negative for infinite)", grid_ttgather,&grid_ttgather );

		if ( ! ok ) {
				printf( "Error in reading number.\n" );
				return;
			}

	}

	intReactivTrans = 0;

	if ( ext_vision_calib_ctx.isTransfValid ) {
		printf("Transf active detected, disabling...\n");
		ext_vision_calib_ctx.isTransfValid = 0;
		intReactivTrans = 1;
	}

	printf("Start collecting calibration data.\n");

	calib_state = mode ? collecting_data_moving : collecting_data_still;

}


static void end_callib_data_col ( void ) {

	if ( isCollectingData() ) {
		calib_state = noOp;
		printf("Stop collecting calibration data.\n");
		usleep( 3.0f / vision_servo_rate );
		if ( intReactivTrans ) {
			printf("Restoring transformation...\n");
			ext_vision_calib_ctx.isTransfValid = 1;
			intReactivTrans = 0;
		}
	}

}


static void reset_observations ( void ) {

	int realloc = 0;

	if ( !get_int( "Reallocate matrices ...", realloc,&realloc ) ) {
		printf( "Error in reading number.\n" );
		return;
	}

	int resume = 0;

	if ( isCollectingData()  ) {
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

	if ( isCollectingData()  )
		end_callib_data_col();

	if ( !get_string("Select filename...", transMFilename,transMFilename ) ) {
		printf( "Error in reading filename.\n" );
		return;
	}

	printf("Importing calibration from %s...",transMFilename);
	int ok = ext_vision_calib_importTransMatrix ( &ext_vision_calib_ctx,transMFilename );
	printf("[%s]\n", ok ? "OK" : "FAIL");

}


static void export_calibration ( void ) {

	if ( isCollectingData()  ) {
		end_callib_data_col();
		calibration_calc();
	}

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

	if ( isCollectingData() ) {
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

	if ( isCollectingData()  ) {
		end_callib_data_col();
		get_int("Want to resume data collection afterwards?",resume,&resume);
	}

	printf("Calculating transformation matrix... ");
	int ok = ext_vision_calib_calcTrans( &ext_vision_calib_ctx );
	printf("%s\n", ok ? "[OK]" : "[FAIL]");

	if ( resume )
		begin_callib_data_col();

}





static int init_vision_calib_task ( void ) {


	calib_state = noOp;

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

	log_init ();

	ext_vision_calib_init( &ext_vision_calib_ctx );

	return ext_vision_comm_init( &ext_vision_ctx );
}



int vision_calib_collect_task ( void ) {

	switch ( calib_state ) {

	case noOp:
		break;

	case collecting_data_still: {

		int isMoving = isRobotMoving();

		if ( !isMoving && vision_up  )
			ext_vision_calib_addPoint( &ext_vision_calib_ctx, &(blobs[1].blob), &(cart_state[1])); //TODO

		break;
	}

	case collecting_data_moving: {

		if (  vision_up  )
			ext_vision_calib_addPoint( &ext_vision_calib_ctx, &(blobs[1].blob), &(cart_state[1]));

		break;
	}

	case collecting_data_grid: {

		static int is_stabilizing = 0;
		static int is_gathering = 0;

		if ( ! isRobotMoving() ) {

			if ( is_stabilizing )
				is_stabilizing -= 1;

			else {

				if ( is_gathering ) {
					is_gathering -= is_gathering < 0 ? 0 : 1;
					if (  vision_up  )
						ext_vision_calib_addPoint( &ext_vision_calib_ctx, &(blobs[1].blob), &(cart_state[1]));
				}

			}

		}
		else {
			is_stabilizing = grid_ttw_stab * vision_servo_rate; //TODO real time, not step time
			is_gathering = grid_ttgather > 0 ? grid_ttgather * vision_servo_rate : -1;
		}

		break;
	}

	}

	return TRUE;
}



int  init_vision_hardware( void)  {

	return init_vision_calib_task();

}


int  acquire_blobs(Blob2D raw_blobs2D[][2+1]) {

	int newData = get_new_vision_data();


	// Outlier detection

	if ( fabs(vision_obs.x[3]) < 0.1 )
		vision_up = 0;


	static SL_Cstate last_vision_data;
//	static int fTime = 0;
//	if ( fTime == 0 ) {
//		fTime = 1;
//		last_vision_data = vision_obs;
//	}
	static int cnt = 0;

	if ( vision_up && cnt < 10 &&
		  ( fabs(vision_obs.x[1]-last_vision_data.x[1]) > 0.15 ||
			fabs(vision_obs.x[2]-last_vision_data.x[2]) > 0.15 ||
			fabs(vision_obs.x[3]-last_vision_data.x[3]) > 0.15 )  ) {
		vision_up = 0;
		cnt += 1;
	}

	if ( vision_up ) {
		cnt = 0;
		last_vision_data = vision_obs;
	}




	// If calibrated apply trans

	SL_Cstate* disp_data = &vision_obs;

	if ( ext_vision_calib_ctx.isTransfValid ) {
		ext_vision_calib_transPoint(&ext_vision_calib_ctx,&vision_obs,&vision_tr);
		disp_data = &vision_tr;
	}



	// Copy to vision_servo structure
	raw_blobs[1].status = vision_up;

	int i;
	for(i=_X_; i<=_Z_; i++)
		raw_blobs[1].x[i] = disp_data->x[i];

	count_all_frames += 1;

	return 1;

}

