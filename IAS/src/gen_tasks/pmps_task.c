#include "SL.h"
#include "SL_man.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_common.h"
#include "SL_dynamics.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"

#include "ias_common.h"

#include "pmps/pmps.h"
#include "pmps/basis/basisGaussNorm.h"
#include "pmps/basis/basisVonMises.h"

#include "ias_motor_utilities.h"
#include "ias_matrix_utilities.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

#include "sharedmemory.h"
#include "SL_episodic_communication.h"
#include "SL_episodic_comm_utils.h"

#include "utility.h"



#ifndef N_DOFS
#define N_DOFS 14
#endif

#pragma message "N_DOFS value: " XSTR(N_DOFS)
#pragma message "N_DOFS_SHM value: " XSTR(N_DOFS_SHM)


enum pmps_state_t {
	noOp = 0,
	gravComp,
	gotoInit,
	goingTo,
	executingCtl,
	executingCtlPos,
	waitingTimeInit,
	waiting
};

typedef struct {

	int isInit;

	int numDim;

	int useInvDyn;

	double dt;
	double Ts;
	double ttw;

	int autoStart;

	Matrix K, k;

	double start_time;

	SL_DJstate joint_init_state[N_DOFS+1];

	int dofMask[N_DOFS+1];

} pmps_task_ctx_s;



static enum pmps_state_t task_state;
static pmps_ctx ctx;
static pmps_task_ctx_s task_ctx;

static Matrix timeM;



static void my_writeStepToSharedMemory ( int idx ) {

	if ( idx > STEPLIMITEPISODE -1 )
		return;

	writeStepToSharedMemoryGeneral(idx);

	int i,j;
	for ( i = 1; i <= task_ctx.numDim; ++i  )
		for ( j = 1; j <= 2*task_ctx.numDim; ++j )
			episodeStep.state[idx][(i-1)*task_ctx.numDim+j-1] = task_ctx.K[i][j];

	for ( i = 1; i <= task_ctx.numDim; ++i  )
			episodeStep.state[idx][2*task_ctx.numDim*task_ctx.numDim+i-1] = task_ctx.k[i][1];
}




static void myFullInvDyn () {


#ifdef DARIAS

	double massMatrix[8][8];

	massMatrix[1][1]=misc_sensor[R_IM11];
	massMatrix[1][2]=misc_sensor[R_IM12];
	massMatrix[1][3]=misc_sensor[R_IM13];
	massMatrix[1][4]=misc_sensor[R_IM14];
	massMatrix[1][5]=misc_sensor[R_IM15];
	massMatrix[1][6]=misc_sensor[R_IM16];
	massMatrix[1][7]=misc_sensor[R_IM17];

	massMatrix[2][1]=misc_sensor[R_IM12];
	massMatrix[2][2]=misc_sensor[R_IM22];
	massMatrix[2][3]=misc_sensor[R_IM23];
	massMatrix[2][4]=misc_sensor[R_IM24];
	massMatrix[2][5]=misc_sensor[R_IM25];
	massMatrix[2][6]=misc_sensor[R_IM26];
	massMatrix[2][7]=misc_sensor[R_IM27];


	massMatrix[3][1]=misc_sensor[R_IM13];
	massMatrix[3][2]=misc_sensor[R_IM23];
	massMatrix[3][3]=misc_sensor[R_IM33];
	massMatrix[3][4]=misc_sensor[R_IM34];
	massMatrix[3][5]=misc_sensor[R_IM35];
	massMatrix[3][6]=misc_sensor[R_IM36];
	massMatrix[3][7]=misc_sensor[R_IM37];


	massMatrix[4][1]=misc_sensor[R_IM14];
	massMatrix[4][2]=misc_sensor[R_IM24];
	massMatrix[4][3]=misc_sensor[R_IM34];
	massMatrix[4][4]=misc_sensor[R_IM44];
	massMatrix[4][5]=misc_sensor[R_IM45];
	massMatrix[4][6]=misc_sensor[R_IM46];
	massMatrix[4][7]=misc_sensor[R_IM47];

	massMatrix[5][1]=misc_sensor[R_IM15];
	massMatrix[5][2]=misc_sensor[R_IM25];
	massMatrix[5][3]=misc_sensor[R_IM35];
	massMatrix[5][4]=misc_sensor[R_IM45];
	massMatrix[5][5]=misc_sensor[R_IM55];
	massMatrix[5][6]=misc_sensor[R_IM56];
	massMatrix[5][7]=misc_sensor[R_IM57];

	massMatrix[6][1]=misc_sensor[R_IM16];
	massMatrix[6][2]=misc_sensor[R_IM26];
	massMatrix[6][3]=misc_sensor[R_IM36];
	massMatrix[6][4]=misc_sensor[R_IM46];
	massMatrix[6][5]=misc_sensor[R_IM56];
	massMatrix[6][6]=misc_sensor[R_IM66];
	massMatrix[6][7]=misc_sensor[R_IM67];

	massMatrix[7][1]=misc_sensor[R_IM17];
	massMatrix[7][2]=misc_sensor[R_IM27];
	massMatrix[7][3]=misc_sensor[R_IM37];
	massMatrix[7][4]=misc_sensor[R_IM47];
	massMatrix[7][5]=misc_sensor[R_IM57];
	massMatrix[7][6]=misc_sensor[R_IM67];
	massMatrix[7][7]=misc_sensor[R_IM77];

	int i,j;
	for (i=1; i<=7; ++i) {
		for (j=1; j<=7; ++j)
			joint_des_state[i].uff +=massMatrix[i][j]*joint_des_state[j].thdd;
	}

#else


	SL_InvDyn( joint_state, joint_des_state, endeff, &base_state, &base_orient ); //TODO?


#endif

}





static void toggle_grav_comp ( void ) {

	task_state = task_state==gravComp ? noOp : gravComp;

}


static void resetLocal ( pmps_task_ctx_s* ctx ) {

	my_basic_free_matrix( ctx->K );
	my_basic_free_matrix( ctx->k );

	ctx->isInit = FALSE;

}

static void set_init_delay ( void ) {
	if ( !  get_double("Set initial in sec:",task_ctx.ttw,&task_ctx.ttw) )
		printf( "Error in reading delay.\n" );
}


static int vonMisesBasis ( double time, int init, char* fnamePrefix ) {

	return FALSE;

	static vonMisesCtx_s basis_ctx;
	static Matrix timeM = 0;

//	if ( init == 1 ){
//
//		my_basic_free_matrix( timeM );
//
//		basisVonMises_free ( &(basis_ctx) );
//
//		timeM = my_matrix( 1, 2, 1, 1 );
//
//		Matrix mu = 0, sigma = 0;
//
//		char fname[1000];
//		strcpy(fname, fnamePrefix);
//		printf("Importing mu from %s...", strcat(fname, "_mu.cf"));
//		int ok = malloc_mat_ascii ( fname, &mu );
//		printf("[%s]\n", ok ? "OK" : "FAIL");
//
//		if ( ok == FALSE )
//			return FALSE;
//
//		strcpy(fname, fnamePrefix);
//		printf("Importing sigma from %s...", strcat(fname, "_pool.cf"));
//		ok = malloc_mat_ascii ( fname, &sigma );
//		printf("[%s]\n", ok ? "OK" : "FAIL");
//
//		if ( ok == FALSE )
//			return FALSE;
//
//		if ( basisVonMises_init ( timeM,  mu,  k, f, &basis_ctx ) == FALSE )
//			return FALSE;
//
//		task_ctx.numBasis = ( (int) sigma[0][NC] ) / task_ctx.numDim;
//
//		my_basic_free_matrix ( mu );
//
//
////		task_ctx.basis   = basis_ctx.basis_n;
////		task_ctx.basisD  = basis_ctx.basisD_n;
////		task_ctx.basisDD = basis_ctx.basisDD_n;
//
//
//		//weights!
//		strcpy(fname, fnamePrefix);
//		printf("Importing w_mu from %s...", strcat(fname, "_w_mu.cf"));
//		ok = malloc_mat_ascii ( fname, &(task_ctx.w_mu) );
//		printf("[%s]\n", ok ? "OK" : "FAIL");
//
//		if ( ok == FALSE )
//			return FALSE;
//
//		strcpy(fname, fnamePrefix);
//		printf("Importing w_cov from %s...", strcat(fname, "_w_cov.cf"));
//		ok = malloc_mat_ascii ( fname, &(task_ctx.w_cov) );
//		printf("[%s]\n", ok ? "OK" : "FAIL");
//
//		if ( ok == FALSE )
//			return FALSE;
//
//	}
//
//	timeM[1][1] = time;
//	timeM[2][1] = time + 1.0 / task_servo_rate;

//	return basisGaussNorm ( timeM, &basis_ctx );
	return FALSE;
}


/* At the first call, initialize
 *
 * must set task_ctx.basis, task_ctx.basisD, task_ctx.basisDD
 * and task_ctx.numBasis
 *
 */
static int gaussianNormBasis ( double time, int init, char* fnamePrefix ) {

	static basisGaussNorm_ctx_s basis_ctx;
	static Matrix timeM = 0;

	if ( init == 1 ){

		my_basic_free_matrix( timeM );

//		basisGaussNorm_free ( &(basis_ctx) );

		timeM = my_matrix( 1, 2, 1, 1 );



		Matrix mu = 0, sigma = 0;
		Matrix w_mu = 0, w_cov = 0;

		char fname[1000];
		strcpy(fname, fnamePrefix);
		printf("Importing mu from %s...", strcat(fname, "_mu.cf"));
		int ok = malloc_mat_ascii ( fname, &mu );
		printf("[%s]\n", ok ? "OK" : "FAIL");

		if ( ok == FALSE )
			return FALSE;

		strcpy(fname, fnamePrefix);
		printf("Importing sigma from %s...", strcat(fname, "_sigma.cf"));
		ok = malloc_mat_ascii ( fname, &sigma );
		printf("[%s]\n", ok ? "OK" : "FAIL");

		if ( ok == FALSE )
			return FALSE;


		strcpy(fname, fnamePrefix);
		printf("Importing w_mu from %s...", strcat(fname, "_w_mu.cf"));
		ok = malloc_mat_ascii ( fname, &w_mu );
		printf("[%s]\n", ok ? "OK" : "FAIL");

		if ( ok == FALSE )
			return FALSE;

		strcpy(fname, fnamePrefix);
		printf("Importing w_cov from %s...", strcat(fname, "_w_cov.cf"));
		ok = malloc_mat_ascii ( fname, &w_cov );
		printf("[%s]\n", ok ? "OK" : "FAIL");

		if ( ok == FALSE )
			return FALSE;


		ok &= calcSysCovBasic_addBasisGauss ( &(ctx.calcSysCov_ctx), timeM, w_mu, w_cov, mu, sigma );

		ctx.calcSysCovF = calcSysCovBasic;



		if ( ok == FALSE )
			return FALSE;


		my_basic_free_matrix ( mu );
		my_basic_free_matrix ( sigma );
		my_basic_free_matrix ( w_mu );
		my_basic_free_matrix ( w_cov );



	}

	timeM[1][1] = time;
	timeM[2][1] = time + 1.0 / task_servo_rate;

	return TRUE;

}



static void setCtlJointsStdIn ( void ) {

	int i;
	for ( i = 1; i <= task_ctx.numDim; ++i ) {
		printf("Set the joint id for the %d dim ",i);
		if ( !  get_int(":",task_ctx.dofMask[i],&(task_ctx.dofMask[i])) )
			printf( "Error in joint id --- reconfigure!\n" );
	}
}


static void setTaskOptionsStdIn ( void ) {

	if ( !  get_int("Enter number of dimensions :",task_ctx.numDim,&(task_ctx.numDim)) )
		printf( "Error please reconfigure!\n" );


	if ( !  get_double("Enter total time :",task_ctx.Ts,&(task_ctx.Ts)) )
		printf( "Error please reconfigure!\n" );


	if ( !  get_double("Enter dt :",task_ctx.dt,&(task_ctx.dt)) )
		printf( "Error please reconfigure!\n" );


	if ( !  get_int("Use inverse dynamics :",task_ctx.useInvDyn,&(task_ctx.useInvDyn)) )
		printf( "Error please reconfigure!\n" );

	task_ctx.useInvDyn = task_ctx.useInvDyn == 0 ? FALSE : TRUE;

	if ( !  get_int("Auto-start task :",task_ctx.autoStart,&(task_ctx.autoStart)) )
		printf( "Error please reconfigure!\n" );

	task_ctx.autoStart = task_ctx.autoStart == 0 ? FALSE : TRUE;

	if ( !  get_double("Enter time to wait from initial state :",task_ctx.ttw,&(task_ctx.ttw)) )
		printf( "Error please reconfigure!\n" );

}




static int setTaskOptionsFile ( void ) {

	int ok = TRUE;
	int i;

	char fPrefix[] = "config/pmps/";
	char fName[1000];


	strcpy (fName, fPrefix);
	strcat (fName, "initPos.cf");
	if ( getFile( fName ) == FALSE ) /* it will change fNanme! */
		return FALSE;

	FILE *file = fopen( fName, "r" );
	if ( file == NULL ) {
		printf( "Problem reading file %s!\n", fName );
		return -1;
	}

	char jointName[100];
	for ( i = 1; i <= n_dofs; ++i ) {
		int jointFound = find_keyword( file, joint_names[i]);
//		printf ( "Looking for %s...[%s]\n", joint_names[i], jointFound ? "OK" : "FAIL" );
		if ( jointFound == TRUE ) {
			int ok = TRUE;
			ok &= fscanf ( file, "%lf", &(task_ctx.joint_init_state[i].th) ) == 1;
//			printf ( "Scanning for %s...[%s]\n", joint_names[i], ok ? "OK" : "FAIL" );
		}
	}

	printf("Importing initial positions from %s...", fName );
	printf("[%s]\n", ok ? "OK" : "FAIL");





	strcpy (fName, fPrefix);
	strcat (fName, "rhythmic_pmps.cf");
	if ( getFile( fName ) == FALSE ) /* it will change fNanme! */
		return FALSE;

	file = fopen( fName, "r" );
	if ( file == NULL ) {
		printf( "Problem reading file %s!\n", fName );
		return -1;
	}



	ok &= find_keyword( file, "numDim");
	ok &= fscanf ( file, "%d", &(task_ctx.numDim)) == 1 ;

	ok &= find_keyword( file, "Ts");
	ok &= fscanf ( file, "%lf", &(task_ctx.Ts)) == 1;

	ok &= find_keyword( file, "dt");
	ok &= fscanf ( file, "%lf", &(task_ctx.dt)) == 1;

	ok &= find_keyword( file, "useInvDyn");
	ok &= fscanf ( file, "%d", &(task_ctx.useInvDyn)) == 1 ;

	task_ctx.useInvDyn = task_ctx.useInvDyn == 0 ? FALSE : TRUE;

	ok &= find_keyword( file, "autoStart");
	ok &= fscanf ( file, "%d", &(task_ctx.autoStart)) == 1 ;

	task_ctx.autoStart = task_ctx.autoStart == 0 ? FALSE : TRUE;

	ok &= find_keyword( file, "ttw");
	ok &= fscanf ( file, "%lf", &(task_ctx.ttw)) == 1;

	ok &= find_keyword( file, "dof_mask");
	for ( i = 1; i <= task_ctx.numDim; ++i ) {
		ok &= fscanf ( file, "%s", jointName ) == 1;
		int j;
	    for (j=1; j<=n_dofs; ++j) {
	      if (strcmp(jointName,joint_names[j]) == 0) {
	    	  task_ctx.dofMask[i] = j;
	    	  break;
	      }
	    }
	    ok &= j <= n_dofs;
	}

//	printf("DOF Mask : numDim %d\n", task_ctx.numDim);
//	for ( i = 1 ; i <= N_DOFS; ++i )
//		printf( "%d ", task_ctx.dofMask[i]);
//	printf("\n");

	ok &= find_keyword( file, "init_pos");
	for ( i = 1; i <= task_ctx.numDim; ++i )
		ok &= fscanf ( file, "%lf", &(task_ctx.joint_init_state[task_ctx.dofMask[i]].th)) == 1;


	int read_sigmas = 0;

	ok &= find_keyword( file, "perTimeStep");
	ok &= fscanf ( file, "%d", &read_sigmas) == 1;

	if ( read_sigmas == 0 ) {
		//init basis currently up to one type

		ok &= find_keyword( file, "numBasisType");

		char basisType[256];
		ok &= find_keyword( file, "basisType");
		ok &= fscanf ( file, "%s", basisType) == 1;


		char basisName[256];
		ok &= find_keyword( file, "basisTypeName");
		ok &= fscanf ( file, " %s", basisName) == 1;

		char basisFullName[256];
		strcpy( basisFullName, fPrefix );
		strcat( basisFullName, basisName );

		if ( strcmp(basisType,"gaussNorm") == 0 ){
			if ( gaussianNormBasis ( 0, 1, basisFullName ) == FALSE ) {
				printf ( "Failed to initialize basis!\n" );
				return FALSE;
			}
		}
		else if ( strcmp(basisType,"vonMises") == 0 ){
			if ( vonMisesBasis ( 0, 1, basisFullName ) == FALSE ) {
				printf ( "Failed to initialize basis!\n" );
				return FALSE;
			}
		}
		else
			ok = FALSE;
	}
	else {

		//init sigmas
		ctx.calcSysCovF = calcSysCovPreAlloc;

		strcpy(fName, fPrefix);
		printf("Importing Sigma_t from %s...", strcat(fName, "rhythmic_ts_sigmat.cf"));
		ok &= malloc_mat_ascii ( fName, &(ctx.calcSysCov_ctx.Sigma_t) );
		printf("[%s]\n", ok ? "OK" : "FAIL");

		//printf("Size of Sigma_t: %d x %d \n", (int) ctx.calcSysCov_ctx.Sigma_t[0][NR], (int) ctx.calcSysCov_ctx.Sigma_t[0][NC] );

		strcpy(fName, fPrefix);
		printf("Importing Mu_t from %s...", strcat(fName, "rhythmic_ts_mu.cf"));
		ok &= malloc_mat_ascii ( fName, &(ctx.calcSysCov_ctx.store_mu_x) );
		printf("[%s]\n", ok ? "OK" : "FAIL");

		strcpy(fName, fPrefix);
		printf("Importing Mu_td from %s...", strcat(fName, "rhythmic_ts_mud.cf"));
		ok &= malloc_mat_ascii ( fName, &(ctx.calcSysCov_ctx.store_mu_xd) );
		printf("[%s]\n", ok ? "OK" : "FAIL");


		strcpy(fName, fPrefix);
		printf("Importing Sigma_t from %s...", strcat(fName, "rhythmic_ts_sigmat.cf"));
		ok &= malloc_mat_ascii ( fName, &(ctx.calcSysCov_ctx.store_Sigma_t) );
		printf("[%s]\n", ok ? "OK" : "FAIL");


		strcpy(fName, fPrefix);
		printf("Importing Sigma_td from %s...", strcat(fName, "rhythmic_ts_sigmatd.cf"));
		ok &= malloc_mat_ascii ( fName, &(ctx.calcSysCov_ctx.store_Sigma_td_half) );
		printf("[%s]\n", ok ? "OK" : "FAIL");


		strcpy(fName, fPrefix);
		printf("Importing Sigma_t_t1 from %s...", strcat(fName, "rhythmic_ts_sigmat_t1.cf"));
		ok &=  malloc_mat_ascii ( fName, &(ctx.calcSysCov_ctx.store_Sigma_t_t1) );
		printf("[%s]\n", ok ? "OK" : "FAIL");


	}



	if ( ok != TRUE )
		printf( "Error please reconfigure!\n" );

	return ok;

}


static void startPmps ( void ) {
	if ( task_state == noOp )
//		task_state = gotoInit;
		task_state = waitingTimeInit;
}


static void graspAstro ( void ) {

	static double Pos[3] = { 57.87, 66.89, 4.13 };

	joint_des_state[27].th = Pos[2] * PI / 180;
	joint_des_state[28].th = Pos[1] * PI / 180;
	joint_des_state[29].th = Pos[0] * PI / 180;
}


static void releaseAstro ( void ) {

	static double Pos[3] = { 7.85, 7.12, 3.88 };

	joint_des_state[27].th = Pos[2] * PI / 180;
	joint_des_state[28].th = Pos[1] * PI / 180;
	joint_des_state[29].th = Pos[0] * PI / 180;
}



static int init_pmps_task ( void ) {

	static int init = 0;

	if ( ! init ) {
		init = 1;

		task_ctx.isInit = FALSE;

		task_ctx.ttw = 1.5;
		task_ctx.autoStart = 0;

		task_ctx.numDim = 1;
		task_ctx.dt = 1.0 / task_servo_rate;
		task_ctx.Ts = 2.0;

		task_ctx.useInvDyn = FALSE;

		task_ctx.K = 0;
		task_ctx.k = 0;

		int i;
		for ( i = 1; i <= N_DOFS; ++i )
			task_ctx.dofMask[i] = i;

		for ( i = 1; i <= N_DOFS; ++i )
			task_ctx.joint_init_state[i].th = joint_state[i].th;

		timeM = my_matrix( 1, 2, 1, 1);

		printf("Task servo rate %d\n", task_servo_rate);


		createEpisodicSharedMemoryCommunication();
		initMotorUtilities(1, 40, 10, task_ctx.joint_init_state, 0.001);

		addToMan( "gravity_comp", "Toggle gravity compensation", toggle_grav_comp );
		addToMan( "set_delay", "Set delay before executing controller", set_init_delay );
		addToMan( "setCtlJoints", "Set controlled joints", setCtlJointsStdIn );
		addToMan( "setTaskOptions", "Set PMPs options", setTaskOptionsStdIn );
		addToMan( "startPmps", "Execute controller", startPmps );

		addToMan( "graspAstro", "Grasp..", graspAstro );
		addToMan( "releaseAstro", "Releas Grasp..", releaseAstro );




	}

	if ( task_ctx.isInit == TRUE ) {
		int aux = 0;
		if ( ! get_int("Re-initialize?",aux,&aux) ) {
			printf( "Error in reading value.\n" );
			return FALSE;
		}
		if ( aux == 0 )
			return go_target_wait( task_ctx.joint_init_state );
	}


	resetLocal ( &task_ctx );



	if ( setTaskOptionsFile() == FALSE )
		return FALSE;

	task_ctx.K = my_matrix ( 1, task_ctx.numDim, 1, 2 * task_ctx.numDim );
	task_ctx.k = my_matrix ( 1, task_ctx.numDim, 1, 1 );


	task_state =  task_ctx.autoStart == TRUE ? waitingTimeInit : noOp;

	if ( pmps_init ( task_ctx.numDim, task_ctx.dt, &ctx ) == FALSE ) { //Should not pass dt...
		printf ( "Failed to initialize PMPs!\n" );
		return FALSE;
	}



	task_ctx.isInit = TRUE;

    if (!go_target_wait( task_ctx.joint_init_state ))
      return FALSE;


	return TRUE;
}

static int run_pmps_task ( void ) {

	static int n_steps;
	static int n_goto_steps;

	static int w_steps;
	static int w_c_step;

	static int ctlIdx = 1;

	static SL_DJstate simState[N_DOFS+1];


	if ( ctx.calcSysCovF == calcSysCovPreAlloc) {
		timeM[1][1] = (int) timeM[1][1]+1;
//		printf("%d ",(int) timeM[1][1] );
	}
	else {
		timeM[1][1] = task_servo_time - task_ctx.start_time;
		timeM[2][1] = task_servo_time - task_ctx.start_time + 1.0 / task_servo_rate;
	}





	switch (task_state) {

	case noOp: {
		int i;
		for ( i = 1; i <= N_DOFS; i++ ) {
//			joint_des_state[i].th = joint_state[i].th;
			joint_des_state[i].thd = 0.0;
			joint_des_state[i].thdd = 0.0;
			joint_des_state[i].uff = 0.0;
		}
//		SL_InvDyn( NULL, joint_des_state, endeff, &base_state, &base_orient );

		break;
	}


	case executingCtl:
	{
		int ok = TRUE;

//		writeStepToSharedMemoryGeneral(ctlIdx++);
		my_writeStepToSharedMemory(ctlIdx++);

		ok &= pmps_execute_step ( &ctx, timeM, task_ctx.K, task_ctx.k );


//		ok &= gaussianNormBasis (task_servo_time - task_ctx.start_time , 0, 0 );
//
//		ok &= pmps_execute_step ( 1, &ctx,
//		                             task_ctx.basis, task_ctx.basisD, task_ctx.basisDD,
//		                             task_ctx.w_mu, task_ctx.w_cov,
//		                             task_ctx.K, task_ctx.k );

		if ( ok == FALSE ) {
			task_state = noOp;
			writeSHMemory(&stepSHM, &episodeStep, &stepSEM);
		}

		int i,j = 0;
		for ( i = 1; i <= N_DOFS; i++ ) {
			joint_des_state[i].th = task_ctx.joint_init_state[i].th;
			joint_des_state[i].thd = 0.0;
			joint_des_state[i].thdd = 0.0;
			joint_des_state[i].uff = 0.0;
		}


		if ( task_ctx.useInvDyn == FALSE) {

			for ( i = 1; i <= task_ctx.numDim; i++ ) {

				joint_des_state[ task_ctx.dofMask[i] ].th  = joint_state[ task_ctx.dofMask[i] ].th;
				joint_des_state[ task_ctx.dofMask[i] ].thd = joint_state[ task_ctx.dofMask[i] ].thd;

				for ( j = 1; j <= task_ctx.numDim; j++ ) {
					joint_des_state[ task_ctx.dofMask[i] ].uff += task_ctx.K[i][j] * joint_state[ task_ctx.dofMask[j] ].th;
					joint_des_state[ task_ctx.dofMask[i] ].uff += task_ctx.K[i][j +task_ctx.numDim ] * joint_state[ task_ctx.dofMask[j] ].thd;
				}
				joint_des_state[ task_ctx.dofMask[i] ].uff += task_ctx.k[i][1];
			}

		}
		else {

			int i,j = 0;
			for ( i = 1; i <= N_DOFS; i++ )
				joint_des_state[i].thdd = (80 * (task_ctx.joint_init_state[i].th  - joint_state[i].th) + 30 * (- joint_state[i].thd))*0.6;

			for ( i = 1; i <= task_ctx.numDim; i++ ) {

				joint_des_state[ task_ctx.dofMask[i] ].th   = joint_state[ task_ctx.dofMask[i] ].th;
				joint_des_state[ task_ctx.dofMask[i] ].thd  = joint_state[ task_ctx.dofMask[i] ].thd;
				joint_des_state[ task_ctx.dofMask[i] ].thdd = gaussian(0, sqrt(ctx.Sigma_u[task_ctx.numDim+i][task_ctx.numDim+i])*task_servo_rate);

				for ( j = 1; j <= task_ctx.numDim; j++ ) {
					joint_des_state[ task_ctx.dofMask[i] ].thdd += task_ctx.K[i][j] * joint_state[ task_ctx.dofMask[j] ].th;
					joint_des_state[ task_ctx.dofMask[i] ].thdd += task_ctx.K[i][j +task_ctx.numDim ] * joint_state[ task_ctx.dofMask[j] ].thd;
				}
				joint_des_state[ task_ctx.dofMask[i] ].thdd += task_ctx.k[i][1];
			}

			SL_InvDyn( joint_state, joint_des_state, endeff, &base_state, &base_orient );
		}


		if ( (task_servo_time - task_ctx.start_time ) > task_ctx.Ts ) {
			task_state = noOp;
			writeSHMemory(&stepSHM, &episodeStep, &stepSEM);
		}

		break;

	}

	case executingCtlPos:
	{
		int ok = TRUE;

		my_writeStepToSharedMemory(ctlIdx++);

		ok &= pmps_execute_step ( &ctx, timeM, task_ctx.K, task_ctx.k );

		if ( ok == FALSE ) {
			task_state = noOp;
			writeSHMemory(&stepSHM, &episodeStep, &stepSEM);
		}

		int i,j = 0;

		for ( i = 1; i <= task_ctx.numDim; i++ ) {



			static double last_noise[10] = {0, 0,0,0,0,0,0,0,0,0};
			static double cTime = 0;
			if ( task_servo_time >  ( cTime + 0.05 ) ) {
				cTime = task_servo_time;
				last_noise[i-1] = gaussian(0, sqrt(ctx.Sigma_u[task_ctx.numDim+i][task_ctx.numDim+i])/0.05);
			}

			simState[ task_ctx.dofMask[i] ].thdd = last_noise[i-1];


			for ( j = 1; j <= task_ctx.numDim; j++ ) {
				simState[ task_ctx.dofMask[i] ].thdd += task_ctx.K[i][j] * simState[ task_ctx.dofMask[j] ].th;
				simState[ task_ctx.dofMask[i] ].thdd += task_ctx.K[i][j +task_ctx.numDim ] * simState[ task_ctx.dofMask[j] ].thd;

			}

			simState[ task_ctx.dofMask[i] ].thdd += task_ctx.k[i][1];

			simState[ task_ctx.dofMask[i] ].th   += simState[ task_ctx.dofMask[i] ].thd / task_servo_rate;
			simState[ task_ctx.dofMask[i] ].thd  += simState[ task_ctx.dofMask[i] ].thdd / task_servo_rate;
		}


		for ( i = 1; i <= 7; i++ ) {
			joint_des_state[i].th   = simState[i].th;
			joint_des_state[i].thd  = simState[i].thd;
			joint_des_state[i].thdd = 0.0;//simState[ task_ctx.dofMask[i] ].thdd; //TODO
			joint_des_state[i].uff  = 0.0;
		}

		for ( i = 1; i <= 7; i++ )
			joint_des_state[i].thdd = (45 * ( simState[i].th  - joint_state[i].th)
					+ 8 * (- joint_state[i].thd));

		for ( i = 1; i <= task_ctx.numDim; i++ ) {
			joint_des_state[i].thdd = simState[ task_ctx.dofMask[i] ].thdd; //TODO
		}

		myFullInvDyn();



//		static my_counter_release = 0;
//		my_counter_release++;
//		if ( my_counter_release >  )
		releaseAstro();

		if ( (task_servo_time - task_ctx.start_time ) > task_ctx.Ts ) {
			task_state = noOp;
			writeSHMemory(&stepSHM, &episodeStep, &stepSEM);
		}

		break;

	}

	case gravComp:
	{
		int i = 0;
		for ( i = 1; i <= 7; i++ ) {  //TODO
			joint_des_state[i].th = joint_state[i].th + joint_state[i].thd / task_servo_rate;
			joint_des_state[i].thd = 0.0;
			joint_des_state[i].thdd = 0.0;
			joint_des_state[i].uff = 0.0;
		}

//		SL_InvDyn( NULL, joint_des_state, endeff, &base_state, &base_orient );

		for ( i = 1; i <= 7; i++ )  //TODO
			joint_des_state[i].thd = joint_state[i].thd;

		check_range( joint_des_state );

		break;
	}


	case gotoInit: {

		task_state = goingTo;

		double max_range=0;

		int i;
		for (i=1; i<= N_DOFS; ++i) {
			double range = fabs( task_ctx.joint_init_state[i].th - joint_state[i].th );
			if (range > max_range)
				max_range = range;
		}
		check_range( task_ctx.joint_init_state );

		/* ensure that the goal state has zero vel and zero acc */
		for (i=1; i<= N_DOFS; ++i)
			task_ctx.joint_init_state[i].thd = task_ctx.joint_init_state[i].thdd = 0.0;


		n_steps = 0;
		n_goto_steps = max_range / 0.25 * task_servo_rate; // goto_speed = 1.0;
		if (n_goto_steps == 0) {
			task_state = waitingTimeInit;
			break;
		}

		for (i=1; i<= N_DOFS; ++i) {
			joint_des_state[i].th = joint_state[i].th;
			joint_des_state[i].thd = joint_state[i].thd;
		}

	}

	case goingTo:
	{

		double time_to_go = (n_goto_steps - n_steps)/((double) task_servo_rate);

		// kinematics follow min jerk
		if ( ! ias_calculate_min_jerk_next_step (joint_des_state, task_ctx.joint_init_state, time_to_go ) ) {
			task_state = noOp;
			printf("Calculate_min_jerk_next_step failed!\n");
			printf("TimeToGo: %lf %d\n",time_to_go,(n_goto_steps - n_steps));
			break;
		}

//		SL_InvDyn( 0, joint_des_state, endeff, &base_state, &base_orient );

		check_range(joint_des_state);

		if (++n_steps >= n_goto_steps-1)
			task_state = waitingTimeInit;

		break;
	}


	case waitingTimeInit: {

		task_state = waiting;
		w_c_step = 0;
		w_steps = task_ctx.ttw * task_servo_rate -1;

		break;
	}


	case waiting: {

		if (++w_c_step >= w_steps) {

			if ( ctx.calcSysCovF == calcSysCovPreAlloc) {
				timeM[1][1] = 0;
			}


//			task_state = executingCtl;

			task_state = executingCtlPos;
			if ( task_state == executingCtlPos ) {



				int i;
				for (i=1; i<=7; ++i) { //TODO
					simState[i].th   = joint_des_state[i].th;
					simState[i].thd  = 0;
					simState[i].thdd = 0;
					simState[i].uff  = 0;
				}
//				for (i=8; i<=n_dofs; ++i) { //TODO
//					simState[i].th   = task_ctx.joint_init_state[i].th;
//					simState[i].thd  = 0;
//					simState[i].thdd = 0;
//					simState[i].uff  = 0;
//				}
			}
			ctlIdx = 1;
			task_ctx.start_time = task_servo_time;
		}

		int i;
		for (i=1; i<=n_dofs; ++i) {
			joint_des_state[i].th = task_ctx.joint_init_state[i].th;
			joint_des_state[i].thd = 0;
		}
//		SL_InvDyn( 0, joint_des_state, endeff, &base_state, &base_orient );

		break;
	}


	}

	return TRUE;

}



static int change_pmps_task ( void ) {

	return TRUE;

}


void add_pmps_task ( void ) {

	addTask( "PMPs_Task", init_pmps_task, run_pmps_task, change_pmps_task );
}
