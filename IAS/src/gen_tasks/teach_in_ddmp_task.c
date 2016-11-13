/*============================================================================
==============================================================================

teach_in_ddmp_task.c

==============================================================================
Remarks:

sekeleton to create the teach_in_ddmp task

============================================================================*/

/* global includes */
#include "SL_system_headers.h"
#include "SL.h"
#include "SL_user.h"
#include "utility.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_man.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_kinematics.h"
#include "SL_filters.h"

/* local includes */
#include "ddmp.h"

/* defines */
static Matrix teach_joints;
static Matrix teach_joints_d;
static Matrix teach_joints_dd;
static int n_pts_max = 0;
static int n_pts = 0;
static int n_llm_max = 0;
static double tau = 0;
static int recording = FALSE;
static int recording_ok = FALSE;
static int playback = FALSE;
static int playback_ok = FALSE;
static int gravity_comp = FALSE;
static int goInit = FALSE;
static double start_time = 0;


static ddmp_x_s ddmp_x;
static ddmp_s ddmp_[N_DOFS+1];

/* local variables */
static SL_DJstate   init_joint_state[N_DOFS+1];

/* local functions */
static int  init_teach_in_ddmp_task(void);
static int  run_teach_in_ddmp_task(void);
static int  change_teach_in_ddmp_task(void);
void start_rec_ddmp();
void stop_rec_ddmp();
void goInit_ddmp();
void playback_ddmp();
void calc_ddmp();
void export_ddmp();
void import_ddmp();
void play();
void record();
void gravity_compensation();
void modify_parameters();



/*****************************************************************************
******************************************************************************
Function Name	: add_teach_in_ddmp_task
Date		: Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_teach_in_ddmp_task( void ) {
	
	addTask("Teach_In_DDMP_Task", init_teach_in_ddmp_task,
			run_teach_in_ddmp_task, change_teach_in_ddmp_task);
	
}    

/*****************************************************************************
******************************************************************************
Function Name	: init_teach_in_ddmp_task
Date		: Dec. 1997

Remarks:

initialization for task

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
static int  init_teach_in_ddmp_task(void)
{
	int i;
	static int firsttime = TRUE;
	
	/* go to a save posture */
	bzero((char *)&(init_joint_state[1]),N_DOFS*sizeof(init_joint_state[1]));
	for (i=1; i<=N_DOFS; i++)
		init_joint_state[i].th = joint_default_state[i].th;
	
	if (!go_target_wait_ID(init_joint_state))
		return FALSE;
	
	if (n_pts_max == 0) 	{
		get_int("Maximal recording time in sec(int)...",60,&n_pts_max);
		n_pts_max *= task_servo_rate;
	}
	
	/* ready to go */
	int ans = 999;
	while (ans == 999) {
		get_int("Enter 1 to start or anthing else to abort ...",ans,&ans);
	}
	
	if (ans != 1) 
		return FALSE;
	

	n_llm_max = (int)ceil(((double)n_pts_max/task_servo_rate)*120/2);
	
	if (firsttime)
	{
		firsttime = FALSE;
		teach_joints = my_matrix(1, n_pts_max, 1, N_DOFS);
		teach_joints_d = my_matrix(1, n_pts_max, 1, N_DOFS);
		teach_joints_dd = my_matrix(1, n_pts_max, 1, N_DOFS);

		ddmp_x_init(&ddmp_x, n_llm_max, "x");

		char varname[30];
		for ( i = 1; i <= N_DOFS; i++ ) {
			sprintf( varname, "DOF_%i", i );
			ddmp_init( &ddmp_[i], &ddmp_x, varname );
		}
	}
	

	ddmp_x.dt = 1./(double)task_servo_rate;

	ddmp_x_reset ( &ddmp_x );
	for ( i = 1; i <= N_DOFS; i++ )
		ddmp_reset( &ddmp_[i] );


	
	start_time = task_servo_time;
	
	addToMan("brp","begin recording data for ddmp",start_rec_ddmp);
	addToMan("erp","end recording data for ddmp",stop_rec_ddmp);
	addToMan("gip","go to initial position of ddmp",goInit_ddmp);
	addToMan("pbp","playback of ddmp",playback_ddmp);
	addToMan("trp","train ddmp",calc_ddmp);
	addToMan("exp","export ddmp",export_ddmp);
	addToMan("imp","import ddmp",import_ddmp);
	addToMan("p","play ddmp",play);
	addToMan("r","record ddmp",record);
	addToMan("gc","gravity compensation on",gravity_compensation);
	addToMan("mod","modify ddmp parameters",modify_parameters);

	
	return TRUE;
}

/*****************************************************************************
******************************************************************************
Function Name	: run_teach_in_ddmp_task
Date		: Dec. 1997

Remarks:

run the task from the task servo: REAL TIME requirements!

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
static int 
run_teach_in_ddmp_task(void)
{
	int j, i;
	
	if (recording && !recording_ok)
	{
		recording_ok = 0;
		for (i=1; i<=N_DOFS; i++)
		{
			if (fabs(joint_state[i].thd)>.1)
			{
				recording_ok--;
			}
		}
		if (recording_ok == 0)
		{
			recording_ok = TRUE;
		}
		else
		{
			recording_ok = FALSE;
			printf("vel !=0, don\'t move yet and try again\n");
			recording = FALSE;
		}
	}
	
	if (recording && recording_ok)
	{
		n_pts++;
		if (n_pts<=n_pts_max)
		{
			for (i=1; i<=N_DOFS; i++)
			{    
				teach_joints[n_pts][i] = joint_state[i].th;
				teach_joints_d[n_pts][i] = joint_state[i].thd;
				teach_joints_dd[n_pts][i] = joint_state[i].thdd;
			}
		}
		else
		{
			printf("max reached\n");
			n_pts--;
			recording = FALSE;
		}
	}
		
	if (gravity_comp && !playback)
	{
		//gravity compensation
		for (i=1; i<=N_DOFS; i++) 
		{    
			joint_des_state[i].th   = joint_state[i].th;
			joint_des_state[i].thd  = 0.0;
			joint_des_state[i].thdd = 0.0;
			joint_des_state[i].uff  = 0.0;      
		}
		
		SL_InvDyn(NULL,joint_des_state,endeff,&base_state,&base_orient);
		for (i=1; i<=N_DOFS; i++)
		{
			joint_des_state[i].thd   = joint_state[i].thd;
		}
	}
	
	if (playback && !playback_ok)
	{
		playback_ok = 0;
		for (i=1; i<=N_DOFS; i++)
		{
			if (fabs(init_joint_state[i].th - joint_state[i].th)>.3 || fabs(joint_state[i].thd)>.1 )
			{
				playback_ok--;
			}
		}
		if (playback_ok == 0)
		{
			playback_ok = TRUE;
		}
		else
		{
			playback_ok = FALSE;
			printf("not in initial position, use gip first\n");
			playback = FALSE;
		}
	}
	
	if (playback && playback_ok)
	{
		ddmp_x_step ( &ddmp_x );
		for (i=1; i<=N_DOFS; i++)
		{
			ddmp_step ( &ddmp_[i] );
			joint_des_state[i].th   = ddmp_[i].y;
			joint_des_state[i].thd  = ddmp_[i].yd;
			joint_des_state[i].thdd = ddmp_[i].ydd;
			joint_des_state[i].uff  = 0.0;
		}
		
		check_range(joint_des_state);
		SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);
		
//		double u_stiction[N_DOFS+1] = {7, 2.7, 2.5, 1.5, 0.725, 0.05, 0.2, 0.01};
//		for(i=1; i<=N_DOFS; i++) {
//			if((fabs(joint_des_state[i].thd) > 0.01) && (fabs(joint_des_state[i].thdd) > 0.1)) {
//				if(joint_des_state[i].thd>0)
//					joint_des_state[i].uff += u_stiction[i];
//				else
//					joint_des_state[i].uff -= u_stiction[i];
//			}
//		}
	}

	static SL_DJstate joint_increment[N_DOFS+1];
	static double max_range;
	static double goto_speed = 1.0;
	static int n_steps;
	static int n_goto_steps;
	static int calc_increment = TRUE;
	
	if (goInit)
	{
		if (calc_increment)
		{
			calc_increment = FALSE;
			max_range = 0;
			
			for (i=1; i<=N_DOFS; i++)
			{
				joint_des_state[i].th = joint_state[i].th;
				if (fabs(init_joint_state[i].th - joint_des_state[i].th) > max_range)
				{
					max_range = fabs(init_joint_state[i].th - joint_des_state[i].th);
				}
			}
			n_steps = 0;
			n_goto_steps = max_range/goto_speed*task_servo_rate;
			
			for (i=1; i<=N_DOFS; i++)
			{
				joint_increment[i].th = (init_joint_state[i].th - joint_des_state[i].th)/(double)n_goto_steps;
			}
		}
		
		if (++n_steps < n_goto_steps)
		{
					for (i=1; i<=n_dofs; i++)
			{
				joint_des_state[i].th   = joint_state[i].th;
//				joint_des_state[i].th   += joint_increment[i].th;
				joint_des_state[i].thd   = joint_state[i].thd;

				double vel = (joint_des_state[i].th + joint_increment[i].th - joint_state[i].th)*task_servo_rate;
				double acc = (vel - joint_state[i].thd)*task_servo_rate;

				joint_des_state[i].thdd  = acc > 20 ? 20 : (acc < -20 ? -20 : acc);
				joint_des_state[i].uff   = 0.0;
			}
		}
		else
		{
			n_steps = n_goto_steps + 10;
		}
		
		check_range(joint_des_state);

		SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);
	}
	else
	{
		calc_increment = TRUE;
	}
	
	return TRUE;
}

/*****************************************************************************
******************************************************************************
Function Name	: change_teach_in_ddmp_task
Date		: Dec. 1997

Remarks:

changes the task parameters

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
static int 
change_teach_in_ddmp_task(void)
{
	int i;
	double temp;
	
	for (i=1; i<=N_DOFS; i++)
	{
		printf("DOF %i",i);		
		get_double(":",init_joint_state[i].th,&temp);
		init_joint_state[i].th  = temp;
	}
	
	check_range(init_joint_state);
	
	return TRUE;
}

void start_rec_ddmp()
{
	int i;
	n_pts = 0;
	playback = FALSE;
	goInit = FALSE;
	if (!gravity_comp)
	{
		printf("move to initial position and call brp again\n");
		gravity_comp = TRUE;
	}
	else if (gravity_comp)
	{
		recording = TRUE;
		recording_ok = FALSE;
		for (i=1; i<=N_DOFS; i++)
		{    
			init_joint_state[i].th   = joint_state[i].th;
			init_joint_state[i].thd  = 0.0;
			init_joint_state[i].thdd = 0.0;     
		}
	}
}

void stop_rec_ddmp()
{
	recording = FALSE;
	playback = FALSE;
	gravity_comp = TRUE;
	goInit = FALSE;
}

void goInit_ddmp()
{
	recording = FALSE;
	playback = FALSE;
	gravity_comp = FALSE;
	goInit = TRUE;
}

void playback_ddmp()
{
	if (tau>0)
	{
		recording = FALSE;
		playback = TRUE;
		gravity_comp = FALSE;
		goInit = FALSE;
		playback_ok = FALSE;
		
		ddmp_x_reset ( &ddmp_x );
		
		int i;
		
		for (i=1; i<=N_DOFS; i++)
		{ 
			ddmp_reset ( &(ddmp_[i]) );
			init_joint_state[i].th   = ddmp_[i].y0;
			init_joint_state[i].thd  = 0.0;
			init_joint_state[i].thdd = 0.0;  
		}
	}
	else
	{
		printf("no ddmp trained yet\n");
	}
}

void calc_ddmp()
{
	int i, j, k;
	static int firsttime = TRUE;
		
	int n_llm = (int)ceil(((double)n_pts/task_servo_rate)*120/2);
	tau = (double)n_pts/task_servo_rate;
	
	static Vector X;
	static Vector fT;
	static Matrix PSI;
	static Vector wNom;
	static Vector wDnom;
	
	if (firsttime)
	{
		firsttime = FALSE;
		X = my_vector(1, n_pts_max);
		fT = my_vector(1, n_pts_max);
		PSI = my_matrix(1, n_llm_max, 1, n_pts_max);
		wNom = my_vector(1, n_llm_max);
		wDnom = my_vector(1, n_llm_max);
	}
	
	////////////////////////////////////////////
	//initialize to desired length
	
	X[0] = n_pts;
	fT[0] = n_pts;
	PSI[0][0] = n_llm;
	PSI[0][1] = n_pts;
	ddmp_x.c[0] = n_llm;
	ddmp_x.h[0] = n_llm;
	ddmp_x.psi[0] = n_llm;
	ddmp_x.n_bases = n_llm;
	wNom[0] = n_llm;
	wDnom[0] = n_llm;
	
	double t=0;
	for (i=1; i<=n_llm; i++)	{
		ddmp_x.c[i]=exp(-ddmp_x.alpha_x*t);
		t+=0.5/(n_llm-1);
	}

	for (i=1; i<=(n_llm-1); i++)
		ddmp_x.h[i]=.5/sqr( (ddmp_x.c[i+1]-ddmp_x.c[i])*0.65 );

	ddmp_x.h[n_llm] = ddmp_x.h[n_llm-1];
	
	for (i=1; i<=N_DOFS; i++)
		ddmp_[i].w[0] = n_llm;


	ddmp_x.dt = 1.0/(double)task_servo_rate;
	ddmp_x.t_total = tau;

	ddmp_x_reset( &ddmp_x );
	
	for (i=1; i<=N_DOFS; i++)
		ddmp_reset( &ddmp_[i] );




	////////////////////////////////////////////
	double tau_l = 0.5/tau;
	for (i=1; i<=N_DOFS; i++)
	{
		printf("DOF %i\n",i);
		ddmp_[i].y0 = teach_joints[1][i];
		ddmp_[i].g = teach_joints[n_pts][i];
		ddmp_[i].gf = ddmp_[i].g;

		double v_max = -1.e30;
		double v_min = +1.e30;
		for (j=1; j<=n_pts; j++)
		{
			if (teach_joints[j][i] > v_max)
				v_max = teach_joints[j][i];

			if (teach_joints[j][i] < v_min)
				v_min = teach_joints[j][i];
		}
		ddmp_[i].a = macro_sign(ddmp_[i].g-ddmp_[i].y0)*(v_max - v_min);
		if (fabs(ddmp_[i].a) < 1.e-6)
			ddmp_[i].a = 1;

		X[1] = 1;
		double xd;
		for (j=2; j<=n_pts; j++)
		{
			xd = -ddmp_x.alpha_x*X[j-1]*tau_l;
			X[j] = X[j-1]+xd*ddmp_x.dt;
		}
		
		for (k=1; k<=ddmp_x.psi[0]; k++)
		{
			for (j=1; j<=n_pts; j++)
			{
				PSI[k][j] = exp(-ddmp_x.h[k]*sqr(X[j]-ddmp_x.c[k]) );
			}
		}

		double amp;
		amp = (ddmp_[i].g-ddmp_[i].y0) + ddmp_[i].a;
		
		for (j=1; j<=n_pts; j++)
		{
			fT[j] = (teach_joints_dd[j][i]/sqr(tau_l)-ddmp_[i].alpha_z*(ddmp_[i].beta_z*(ddmp_[i].g-teach_joints[j][i])-teach_joints_d[j][i]/tau_l))/amp;
		}
		
		for (k=1; k<=ddmp_[i].w[0]; k++)
		{
			wDnom[k] = 0;
			for (j=1; j<=n_pts; j++)
			{
				wDnom[k] += PSI[k][j]*sqr(X[j]);
			}
		}
		for (k=1; k<=ddmp_[i].w[0]; k++)
		{
			wNom[k] = 0;
			for (j=1; j<=n_pts; j++)
			{
				wNom[k] += PSI[k][j]*X[j]*fT[j];
			}
		}
		
		for (k=1; k<=ddmp_[i].w[0]; k++)
		{
		ddmp_[i].w[k] = wNom[k] / (wDnom[k]+1.e-10);
		}
		
		ddmp_reset( &ddmp_[i] );
	}	
}

void export_ddmp()
{
	int i, k;
	FILE *file;
	file = fopen("w.txt","w");
	for (k=1; k<=ddmp_[1].w[0]; k++)
	{
		for (i=1; i<=N_DOFS; i++)
			fprintf(file,"%lf ",ddmp_[i].w[k]);

		fprintf(file,"\n");
	}
	fclose(file);
	
	file = fopen("param.txt","w");
	fprintf(file,"%f %f \n", tau, ddmp_[1].w[0]);
		
	for (i=1; i<=N_DOFS; i++)
		fprintf(file,"%lf ",ddmp_[i].y0);
	fprintf(file,"\n");
	
	for (i=1; i<=N_DOFS; i++)
		fprintf(file,"%lf ",ddmp_[i].g);
	fprintf(file,"\n");
	
	for (i=1; i<=N_DOFS; i++)
		fprintf(file,"%lf ",ddmp_[i].a);
	fprintf(file,"\n");
	
	fclose(file);
}

void import_ddmp()
{
	recording = FALSE;
	playback = FALSE;
	gravity_comp = TRUE;
	goInit = FALSE;
	
	int i, k;
	FILE *stream;
	double read[N_DOFS+1];
	double n_llm;
	
	if ( (stream = fopen( "param.txt","r")) == NULL) {
		printf("Can't open %s\n","param.txt");
		return;
	}
	
	if ( fscanf(stream, "%lf %lf \n", &tau, &n_llm) == -1 )
		printf("End of file\n");
	else
	{
		for (i=1; i<=N_DOFS; i++)
		{
			ddmp_[i].w[0] = (int)n_llm;
		}
		ddmp_x.c[0] = (int)n_llm;
		ddmp_x.h[0] = (int)n_llm;
		ddmp_x.psi[0] = (int)n_llm;

		
		ddmp_x_reset ( &ddmp_x );
		ddmp_x.t_total = tau;
		ddmp_x.dt = 1.0/(double)task_servo_rate;

		for (i=1; i<=N_DOFS; i++)
			ddmp_reset( &ddmp_[i]);


		double t=0;
		for (i=1; i<=(int)n_llm; i++)
		{
			ddmp_x.c[i]=exp(-ddmp_x.alpha_x*t);
			t+=0.5/(n_llm-1);
		}
		for (i=1; i<=((int)n_llm-1); i++)
		{
			ddmp_x.h[i]=.5/sqr( (ddmp_x.c[i+1]-ddmp_x.c[i])*0.65 );
		}
		ddmp_x.h[(int)n_llm] = ddmp_x.h[(int)n_llm-1];
	}


    double tmp;
    for (i = 1; i <= N_DOFS; i++) {
		if ( fscanf(stream, "%lf ", &tmp) == -1 ) {
			printf("End of file\n");
			break;
		}
		ddmp_[i].y0 = tmp;
    }
    fscanf(stream, "\n");


    for (i = 1; i <= N_DOFS; i++) {
		if ( fscanf(stream, "%lf ", &tmp) == -1 ) {
			printf("End of file\n");
			break;
		}
		ddmp_[i].g = tmp;
    }
    fscanf(stream, "\n");


    for (i = 1; i <= N_DOFS; i++) {
		if ( fscanf(stream, "%lf ", &tmp) == -1 ) {
			printf("End of file\n");
			break;
		}
		ddmp_[i].a = tmp;
    }
    fscanf(stream, "\n");

	fclose(stream);


	if ( (stream = fopen( "w.txt","r")) == NULL) {
		printf("Can't open %s\n","w.txt");
		return;
	}



	for (k=1; k<=ddmp_[1].w[0]; k++)
	{
	    for (i = 1; i <= N_DOFS; i++) {
			if ( fscanf(stream, "%lf ", &tmp) == -1 ) {
				printf("End of file\n");
				break;
			}
			ddmp_[i].w[k] = tmp;
	    }
	    fscanf(stream, "\n");
	}
	fclose(stream);




	for (i=1; i<=N_DOFS; i++)
	{ 
		init_joint_state[i].th   = ddmp_[i].y0;
		init_joint_state[i].thd  = 0.0;
		init_joint_state[i].thdd = 0.0;  
	}
	goInit_ddmp();
}



void play()
{	
	int ans = 1;
	while (ans != 0)
	{
		goInit_ddmp();
		get_int("press <enter> for play back ...",ans,&ans);
		playback_ddmp();
		ans = 1;
		get_int("press <enter> to replay (exit=0)",ans,&ans);
	}
}

void record()
{
	int ans = 1;
	while (ans != 0)
	{
		start_rec_ddmp();
		get_int("press <enter> if initial position is ok ...",ans,&ans);
		printf("recording\n");
		start_rec_ddmp();
		get_int("press <enter> to stop recording ...",ans,&ans);
		stop_rec_ddmp();
		//get_int("press <enter> to go to initial position ...",ans,&ans);
		//goInit_ddmp();
		ans = 1;
		get_int("press <enter> to rerecord (exit=0)",ans,&ans);
	}
}

void gravity_compensation()
{
	playback = FALSE;
	goInit = FALSE;
	gravity_comp = TRUE;
}

void modify_parameters() {
	double t_scale;
	get_double("Set time scale",ddmp_x.t_total,&t_scale);
	ddmp_x.t_total /= t_scale;
	//ddmp_x_reset( &ddmp_x );
	
	int j = 0;
	get_int("Select Joint [0 to exit]",j,&j);
	if ( j == 0 )
		return;
	get_double("Set new goal",ddmp_[j].gf,&ddmp_[j].gf);
	//ddmp_reset(&ddmp_[j]);
	
}
