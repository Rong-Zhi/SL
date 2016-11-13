/***********************************************************************
 * This file contains the main functions to simulate the system, i.e.
 * the state transition function for the ball position.
 **********************************************************************/
#include "SL_system_headers.h"
#include "SL.h"
#include "SL_user.h"
#include "SL_task_servo.h"
#include "SL_collect_data.h"
#include "SL_dynamics.h"
#include "quat_tools.h"

#include "utility.h"
#include "beerpong.h"
#include "beerpong_env.h"

SL_Cstate cup_state;
SL_Cstate ball_state;

static SL_quat cup_orient;
static SL_quat orientBack;
static SL_quat orientBackHand;
static SL_quat orientForward;
static SL_quat orientForwardHand;
static SL_Cstate launcher_state; // the simulator considers the hand of the robot as a ball launcher
static SL_quat launcher_orient;
static SL_quat launcher_rot;

static int b_rim_top = FALSE;
static int b_rim_bot = FALSE;
static int b_top = FALSE;
static int b_bot = FALSE;
static int b_boti = FALSE;
static int b_ins = FALSE;
static int b_outs = FALSE;

static double CRT = .7;

static int stiction = TRUE;
static int check_on_launcher = TRUE;

static double rvec_b[4];
static double vec_b[4];
static double rvec[4];
static double vec[4];


int add_beerpong_vars()
{
	int i;
	char varname[30];
	for (i = _X_; i <= _Z_; i++)
	{				// Cup-Velocities
		sprintf (varname, "dx_cup_%d", i);
		addVarToCollect ((char *) &(cup_state.xd[i]), varname, "m/s", DOUBLE,
						 FALSE);
	};
	for (i = _X_; i <= _Z_; i++)
	{				// Cup-Accelerations
		sprintf (varname, "ddx_cup_%d", i);
		addVarToCollect ((char *) &(cup_state.xdd[i]), varname, "m/s^2", DOUBLE,
						 FALSE);
	};
	for (i = _Q0_; i <= _Q3_; i++)
	{				// Cup-Orientation
		sprintf (varname, "cup_q_%d", i);
		addVarToCollect ((char *) &(cup_orient.q[i]), varname, "E", DOUBLE,
						 FALSE);
	};
	
	for (i = _X_; i <= _Z_; i++)
	{				// Ball-Positions
		sprintf (varname, "x_ball_%d", i);
		addVarToCollect ((char *) &(blobs[1].blob.x[i]), varname, "m", DOUBLE,
						 FALSE);
	};
	for (i = _X_; i <= _Z_; i++)
	{				// Ball_Velocities
		sprintf (varname, "dx_ball_%d", i);
		addVarToCollect ((char *) &(blobs[1].blob.xd[i]), varname, "m/s", DOUBLE,
						 FALSE);
	};
	for (i = _X_; i <= _Z_; i++)
	{				// Ball-Accelerations
		sprintf (varname, "ddx_ball_%d", i);
		addVarToCollect ((char *) &(blobs[1].blob.xdd[i]), varname, "m/s^2",
						 DOUBLE, FALSE);
	};	
	
	for (i = _X_; i <= _Z_; i++)
	{				// Sim-Ball-Positions
		sprintf (varname, "x_ball_sim_%d", i);
		addVarToCollect ((char *) &(ball_state.x[i]), varname, "m", DOUBLE,FALSE);
	};
	for (i = _X_; i <= _Z_; i++)
	{				// Sim-Ball_Velocities
		sprintf (varname, "dx_ball_sim_%d", i);
		addVarToCollect ((char *) &(ball_state.xd[i]), varname, "m/s", DOUBLE,FALSE);
	};
	for (i = _X_; i <= _Z_; i++)
	{				// Sim-Ball-Accelerations
		sprintf (varname, "ddx_ball_sim_%d", i);
		addVarToCollect ((char *) &(ball_state.xdd[i]), varname, "m/s^2", DOUBLE, FALSE);
	};
	
	addVarToCollect ((char *) &(stiction), "stiction", "-", INT, FALSE);
	addVarToCollect ((char *) &(check_on_launcher), "check_on_launcher", "-", INT, FALSE);
	
	return TRUE;
}


void init_beerpong_state()
{
	endeff[RIGHT_HAND].x[3] = 0.2;
	int dim = 0;
			// Rotate Cup
	cup_orient.q[_Q0_] = cos(0./2.);
	cup_orient.q[_Q1_] = 1.*sin(0./2.);
	cup_orient.q[_Q2_] = 1.*sin(0./2.);
	cup_orient.q[_Q3_] = 1.*sin(0./2.);
		
	double norm = 0.;
	for (dim = _Q0_; dim <= _Q3_; dim++)
	{
		norm += sqr(cup_orient.q[dim]);
	}
	norm = sqrt(norm);
	for (dim = _Q0_; dim <= _Q3_; dim++)
	{
		cup_orient.q[dim] /= norm;
	}
		
	launcher_rot.q[_Q0_] = cos(PI/2.);
	launcher_rot.q[_Q1_] = 0.*sin(PI/2.);
	launcher_rot.q[_Q2_] = 0.*sin(PI/2.);
	launcher_rot.q[_Q3_] = 1.*sin(PI/2.);
		
	for (dim = _X_; dim <= _Z_; dim++)
	{
		launcher_state.x[dim] = cart_state[RIGHT_HAND].x[dim];
		launcher_state.xd[dim] = cart_state[RIGHT_HAND].xd[dim];
		launcher_state.xdd[dim] = cart_state[RIGHT_HAND].xdd[dim];
	};
	for (dim = _Q0_; dim <= _Q3_; dim++)
	{
		launcher_orient.q[dim] = cart_orient[RIGHT_HAND].q[dim];
	};
	mult_quat(&launcher_orient, &launcher_rot, &launcher_orient);
	
	// quaternions for launcher rotation
	for (dim = _Q0_; dim <= _Q3_; dim++)
	{
		orientForwardHand.q[dim] = launcher_orient.q[dim];
		orientBackHand.q[dim] = launcher_orient.q[dim];
	};
  	orientForwardHand.q[1] = cos(PI-acos(launcher_orient.q[1]));
				
	rvec[_X_] = 0.0;
	rvec[_Y_] = 0.0;
	rvec[_Z_] = ball_radius;
	rotate_quat(orientBackHand, rvec, vec);

	for (dim = _X_; dim <= _Z_; dim++)
	{
		ball_state.x[dim] = launcher_state.x[dim] + vec[dim];
		ball_state.xd[dim] = launcher_state.xd[dim];
		ball_state.xdd[dim] = launcher_state.xdd[dim];
	}
		
	stiction = TRUE;
	check_on_launcher = TRUE;
	
	// cup at fixed location
	cup_state.x[_X_] = 0.;
	cup_state.x[_Y_] = -2.43;
	cup_state.x[_Z_] = floor_level+table_height+table_thickness;
		
	for (dim = _X_; dim <= _Z_; dim++) 
	{
		cup_state.xd[dim]  = 0;
		cup_state.xdd[dim] = 0;
	};
}


int sim_beerpong_state(double bounceNoise)
{
	double time_step = 1. / (double)task_servo_rate;
	int i, dim;
	int bounce = FALSE;
	double diff_x[4], diff_xd[4];
	static double rvec_a[4], vec_a[4], rvec_db[4], vec_db[4], rvec_ddb[4], vec_ddb[4], rvec_g[4], vec_g[4], rvec_v[4], vec_v[4];
	
	static SL_quat cup_rot;
		
	double ball_rxy, ball_rxy_P1, rvec_b_Z_P1, eps;
	
	for (dim = _X_; dim <= _Z_; dim++)
	{
		launcher_state.x[dim] = cart_state[RIGHT_HAND].x[dim];
		launcher_state.xd[dim] = cart_state[RIGHT_HAND].xd[dim];
		launcher_state.xdd[dim] = cart_state[RIGHT_HAND].xdd[dim];
	};
	for (dim = _Q0_; dim <= _Q3_; dim++)
	{
		launcher_orient.q[dim] = cart_orient[RIGHT_HAND].q[dim];
	};
	mult_quat(&launcher_orient, &launcher_rot, &launcher_orient);
	
	// quaternions for launcher rotation
	for (dim = _Q0_; dim <= _Q3_; dim++)
	{
		orientForwardHand.q[dim] = launcher_orient.q[dim];
		orientBackHand.q[dim] = launcher_orient.q[dim];
	};
  	orientForwardHand.q[1] = cos(PI-acos(launcher_orient.q[1]));
		
	// quaternions for cup rotation
	rvec_a[_X_] = 0;
	rvec_a[_Y_] = 0;
	rvec_a[_Z_] = cup_z + cup_height;
	
	rotate_quat(cup_orient, rvec_a, vec_a);
	
	for (dim = _Q0_; dim <= _Q3_; dim++)
	{
		orientForward.q[dim] = cup_orient.q[dim];
		orientBack.q[dim] = cup_orient.q[dim];
	};
  
	orientForward.q[1] = cos(PI-acos(cup_orient.q[1]));
	
	if (stiction)
	{
		rvec[_X_] = 0.0;
		rvec[_Y_] = 0.0;
		rvec[_Z_] = ball_radius;
		rotate_quat(orientBackHand, rvec, vec);
		for (dim = _X_; dim <= _Z_; dim++)
		{
			ball_state.x[dim] = launcher_state.x[dim] + vec[dim];
			ball_state.xd[dim] = launcher_state.xd[dim];
			ball_state.xdd[dim] = launcher_state.xdd[dim];
		}
		vec_g[_X_] = 0.0;
		vec_g[_Y_] = 0.0;
		vec_g[_Z_] = -gravity;
		rotate_quat(orientForwardHand, vec_g, rvec_g);
		rotate_quat(orientForwardHand, launcher_state.xdd, rvec);
		if (rvec_g[_Z_]>rvec[_Z_])
		{
			stiction = FALSE;
		}
		else
		{
			if (fabs((rvec_g[_Z_]-rvec[_Z_])*.5)<fabs(rvec_g[_X_]-rvec[_X_]) )
			{
				stiction = FALSE;
			}
		}
	}
	
	if (!stiction)
	{
		if (check_on_launcher)
		{
			// ballistic flight
			for (dim = _X_; dim <= _Z_; dim++)
			{
				ball_state.xdd[dim] = 0.;
			}
			ball_state.xdd[_Z_] -= gravity;
			
			// integrate over Time
			for (dim = _X_; dim <= _Z_; dim++)
			{
				ball_state.xd[dim] += ball_state.xdd[dim] * time_step;	//Euler explizit
				ball_state.x[dim] += ball_state.xd[dim] * time_step;	//Euler implizit -> Stabilität!!!
			}
			
			for (dim = _X_; dim <= _Z_; dim++)
			{
				vec[dim] = ball_state.x[dim] - launcher_state.x[dim];
				vec_v[dim] = ball_state.xd[dim] - launcher_state.xd[dim];
			}
			rotate_quat(orientForwardHand, vec, rvec);
			rotate_quat(orientForwardHand, vec_v, rvec_v);
			
			if (fabs(rvec[_X_])<launcher_w/2. && fabs(rvec[_Y_])<launcher_l/2. && rvec[_Z_]<launcher_w && rvec[_Z_]>-(ball_radius+launcher_h))
			{
				rvec[_X_] = 0.;
				rvec[_Z_] = ball_radius;
				if (rvec[_Y_]<=0.)
				{
					rvec[_Y_] = 0.;
					stiction = TRUE;
				}
				rotate_quat(orientBackHand, rvec, vec);
				for (dim = _X_; dim <= _Z_; dim++)
				{
					ball_state.x[dim] = launcher_state.x[dim] + vec[dim];
				}
				
				rvec_v[_X_] = 0.;
				rvec_v[_Z_] = 0.;
				if (stiction)
				{
					rvec_v[_Y_] = 0.;
				}
				rotate_quat(orientBackHand, rvec_v, vec_v);
				for (dim = _X_; dim <= _Z_; dim++)
				{
					ball_state.xd[dim] = launcher_state.xd[dim] + vec_v[dim];
				}
				
				for (dim = _X_; dim <= _Z_; dim++)
				{
					vec[dim] = ball_state.xdd[dim] - launcher_state.xdd[dim];
				}
				rotate_quat(orientForwardHand, vec, rvec);
				rvec[_X_] = 0.;
				rvec[_Z_] = 0.;
				if (stiction)
				{
					rvec[_Y_] = 0.;
				}
				rotate_quat(orientBackHand, rvec, vec);
				for (dim = _X_; dim <= _Z_; dim++)
				{
					ball_state.xdd[dim] = launcher_state.xdd[dim] + vec[dim];
				}
			}
			else
			{
				check_on_launcher = FALSE;
			}
		}
		else
		{
			// calculate the distance to the cup
			for (dim = _X_; dim <= _Z_; dim++)
			{
				vec_b[dim] = ball_state.x[dim] - cup_state.x[dim];
			};
			
			rotate_quat(orientForward, vec_b, rvec_b);
			
			ball_rxy = sqrt( sqr(rvec_b[_X_]) + sqr(rvec_b[_Y_]) );
			
			// set xdd
			for (dim = _X_; dim <= _Z_; dim++)
			{
				ball_state.xdd[dim] =0.;
			};
			if(ball_state.x[_Z_]>floor_level+ball_radius)
			{
				ball_state.xdd[_Z_] -= gravity;	
			}
			else
			{
				for (dim = _X_; dim <= _Z_; dim++)
				{
					ball_state.xd[dim] = 0.;
				}
			}
			
			// bounce
			for (dim = _X_; dim <= _Z_; dim++)
			{
				rvec_ddb[dim] = 0;
			};
			
			for (dim = _X_; dim <= _Z_; dim++)
			{
				vec_db[dim] = ball_state.xd[dim] - cup_state.xd[dim];
			};
			
			rotate_quat(orientForward, vec_db, rvec_db);
			
			ball_rxy_P1 = sqrt(sqr(rvec_b[_X_]+rvec_db[_X_]*time_step) + sqr(rvec_b[_Y_]+rvec_db[_Y_]*time_step));
			rvec_b_Z_P1 = rvec_b[_Z_]+rvec_db[_Z_]*time_step;

			// outside top
			if ( (sqr(ball_rxy-(cup_radius-cup_rim))+sqr(rvec_b[_Z_]-(cup_height+cup_z-cup_rim)))<sqr(ball_radius+cup_rim) && ball_rxy>(cup_radius-cup_rim) )
			{
				double nZ = ball_rxy*(rvec_b[_Z_]-(cup_height+cup_z-cup_rim))/(ball_rxy-(cup_radius-cup_rim));
				double u = -2.0 * ( rvec_b[_X_]*rvec_db[_X_] + rvec_b[_Y_]*rvec_db[_Y_] + nZ*rvec_db[_Z_]) / ( sqr(rvec_b[_X_])+sqr(rvec_b[_Y_])+sqr(nZ) );
				
				for (dim = _X_; dim <= _Z_; dim++)
				{
					vec[dim] = cup_state.xd[dim];
				};
				rotate_quat(orientForward, vec, rvec);
				
				rvec_ddb[_X_] = (1+restitution)/(2*time_step)*(u*rvec_b[_X_]+rvec[_X_]);
				rvec_ddb[_Y_] = (1+restitution)/(2*time_step)*(u*rvec_b[_Y_]+rvec[_Y_]);
				rvec_ddb[_Z_] = (1+restitution)/(2*time_step)*(u*nZ+rvec[_Z_]);
				bounce = TRUE;
				b_rim_top = TRUE;
			}
			else
				b_rim_top = FALSE;
			// outside bottom
			if ( (sqr(ball_rxy-(cup_radius-cup_rim))+sqr(rvec_b[_Z_]-(cup_z+cup_rim)))<sqr(ball_radius+cup_rim) && ball_rxy>(cup_radius-cup_rim) )
			{
				double nZ = ball_rxy*(rvec_b[_Z_]-(cup_z+cup_rim))/(ball_rxy-(cup_radius-cup_rim));
				double u = -2.0 * ( rvec_b[_X_]*rvec_db[_X_] + rvec_b[_Y_]*rvec_db[_Y_] + nZ*rvec_db[_Z_]) / ( sqr(rvec_b[_X_])+sqr(rvec_b[_Y_])+sqr(nZ) );
				
				for (dim = _X_; dim <= _Z_; dim++)
				{
					vec[dim] = cup_state.xd[dim];
				};
				rotate_quat(orientForward, vec, rvec);
				
				rvec_ddb[_X_] = (1+restitution)/(2*time_step)*(u*rvec_b[_X_]+rvec[_X_]);
				rvec_ddb[_Y_] = (1+restitution)/(2*time_step)*(u*rvec_b[_Y_]+rvec[_Y_]);
				rvec_ddb[_Z_] = (1+restitution)/(2*time_step)*(u*nZ+rvec[_Z_]);
				bounce = TRUE;
				b_rim_bot = TRUE;
			}
			else
				b_rim_bot = FALSE;
			
			// outside
			if ( ball_rxy>=cup_radius && ball_rxy_P1<=(cup_radius+ball_radius)
					&& ( ( rvec_b[_Z_]>=(cup_z + cup_rim) && rvec_b[_Z_]<=(cup_z + cup_height - cup_rim) )
						|| ( rvec_b_Z_P1>=(cup_z + cup_rim) && rvec_b_Z_P1<=(cup_z + cup_height - cup_rim) ) ) )
			{
				double u = -2.0 * ( rvec_b[_X_]*rvec_db[_X_] + rvec_b[_Y_]*rvec_db[_Y_] ) / ( sqr(rvec_b[_X_])+sqr(rvec_b[_Y_]) );
				
				for (dim = _X_; dim <= _Z_; dim++)
				{
					vec[dim] = cup_state.xd[dim];
				};
				rotate_quat(orientForward, vec, rvec);
				
				rvec_ddb[_X_] = (1+restitution)/(2*time_step)*(u*rvec_b[_X_]+rvec[_X_]);
				rvec_ddb[_Y_] = (1+restitution)/(2*time_step)*(u*rvec_b[_Y_]+rvec[_Y_]);
				rvec_b[_X_] = (cup_radius+ball_radius)/ball_rxy*rvec_b[_X_];
				rvec_b[_Y_] = (cup_radius+ball_radius)/ball_rxy*rvec_b[_Y_];
				bounce = TRUE;
				b_outs = TRUE;
			}
			else b_outs = FALSE;

			// inside
			if ( ball_rxy<=(cup_radius-cup_wall) && ball_rxy_P1>=(cup_radius-ball_radius-cup_wall)
					&& ( ( rvec_b[_Z_]>(cup_z) && rvec_b[_Z_]<(cup_z + cup_height) ) 
						|| ( rvec_b_Z_P1>(cup_z) && rvec_b_Z_P1<(cup_z + cup_height) ) ) )
			{
				double u = -2.0 * ( rvec_b[_X_]*rvec_db[_X_] + rvec_b[_Y_]*rvec_db[_Y_] ) / ( sqr(rvec_b[_X_])+sqr(rvec_b[_Y_]) );
				
				for (dim = _X_; dim <= _Z_; dim++)
				{
					vec[dim] = cup_state.xd[dim];
				};
				rotate_quat(orientForward, vec, rvec);
				
				rvec_ddb[_X_] = (1+restitution)/(2*time_step)*(u*rvec_b[_X_]+rvec[_X_]);
				rvec_ddb[_Y_] = (1+restitution)/(2*time_step)*(u*rvec_b[_Y_]+rvec[_Y_]);
				rvec_b[_X_] = (cup_radius-cup_wall-ball_radius)/ball_rxy*rvec_b[_X_];
				rvec_b[_Y_] = (cup_radius-cup_wall-ball_radius)/ball_rxy*rvec_b[_Y_];
				bounce = TRUE;
				b_ins = TRUE;
			}
			else
				b_ins = FALSE;

			// bottom
			if ( rvec_b[_Z_]<=cup_z && rvec_b_Z_P1>=(cup_z - ball_radius)
					&& ( ball_rxy<=(cup_radius-cup_rim) || ball_rxy_P1<=(cup_radius-cup_rim) ) )
			{
				rvec_ddb[_Z_] = -(1+restitution)*rvec_db[_Z_]/time_step;
				rvec_b[_Z_]  = (cup_z - ball_radius);
				bounce = TRUE;
				b_bot = TRUE;
			}
			else
				b_bot = FALSE;

			// top
			if ( rvec_b[_Z_]>=(cup_z + cup_height) && rvec_b_Z_P1<=(cup_z + cup_height + ball_radius)
					&& ( ( ball_rxy>(cup_radius-cup_wall) && ball_rxy<=(cup_radius-cup_rim) )
						|| ( ball_rxy_P1>(cup_radius-cup_wall) && ball_rxy_P1<=(cup_radius-cup_rim) ) ) )
			{
				rvec_ddb[_Z_] = -(1+restitution)*rvec_db[_Z_]/time_step;
				rvec_b[_Z_]  = (cup_z + ball_radius + cup_height);
				bounce = TRUE;
				b_top = TRUE;
			}
			else
				b_top = FALSE;

			// inside bottom
			if ( rvec_b[_Z_]>=(cup_z + cup_wall ) && rvec_b_Z_P1<=(cup_z + cup_wall + ball_radius) 
					&& ( ball_rxy<=(cup_radius - cup_wall) || ball_rxy_P1<=(cup_radius - cup_wall) ) )
			{
				rvec_ddb[_X_] = 0;
				rvec_ddb[_Y_] = 0;
				rvec_ddb[_Z_] = 0;
				rvec_db[_X_] = 0;
				rvec_db[_Y_] = 0;
				rvec_db[_Z_] = 0;
				rvec_b[_X_] = 0;
				rvec_b[_Y_] = 0;
				rvec_b[_Z_] = (cup_z + ball_radius + cup_wall);
				//printf("Yeah! In the cup!\n");
				bounce = TRUE;
				b_boti = TRUE;
			}
			else
				b_boti = FALSE;

			// change x & xdd for bounce
			if (bounce)
			{
				rotate_quat(orientBack, rvec_ddb, vec_ddb);  
				rotate_quat(orientBack, rvec_b, vec_b);
				for(dim=_X_; dim<=_Z_; dim++)
				{
					ball_state.xdd[dim] = vec_ddb[dim]; 
					ball_state.x[dim] = cup_state.x[dim] + vec_b[dim];
				};
			};
			
			// check if over or under table
			if( (fabs(ball_state.x[_Y_] - (dist_to_table-.5*table_length)) <= .5*table_length) 
				   && (fabs(ball_state.x[_X_]-table_center) <= .5*table_width)) 
			{
				// distance over table:
				eps = ball_state.x[_Z_] - (floor_level + table_height + table_thickness + ball_radius);
				
				if( eps<=0 && eps>=-.75*table_thickness && ball_state.xd[_Z_] <= 0.)
				{ 
					// reflect to top

					ball_state.xd[_Z_] =  -(CRT * 1.0 + gaussian(0.0, bounceNoise))*ball_state.xd[_Z_];
					ball_state.x[_Z_] = floor_level + table_height + table_thickness + ball_radius;
				}
				else
				{
					eps = ball_state.x[_Z_] - (floor_level + table_height - ball_radius);
					if( eps>=0 && eps<=-.75*table_thickness && ball_state.xd[_Z_] >= 0.)
					{ 
						// reflect to bottom
						ball_state.xd[_Z_] = -CRT*ball_state.xd[_Z_];
						ball_state.x[_Z_] = floor_level + table_height - ball_radius;			
					}
				}
			}
			
			// integrate over time
			for (dim = _X_; dim <= _Z_; dim++)
			{
				ball_state.xd[dim] += ball_state.xdd[dim] * time_step;	// Euler esplicit
				ball_state.x[dim]  += ball_state.xd[dim] * time_step;	// Euler implicit -> stability!!!
			};
			
			if( ball_state.x[_Z_]<(floor_level + ball_radius) )
			{
				ball_state.x[_Z_] = floor_level + ball_radius;
				for(dim = _X_; dim <= _Z_; dim++) 
				{
					ball_state.xd[dim] = 0.;
					ball_state.xdd[dim] = 0.;
				}	
			}
			
		}
	}
	return TRUE;
}

void send_beerpong_graphics()
{
	// you can choose to use the vision system (commented below) to display the ball
	// or draw it manually (uncommented below) using the position got from the simulator
	
	// static int last_frame_counter = -999;
	// 
	// if (last_frame_counter != frame_counter)
	// {
		// raw_blobs[BLOB1].status = TRUE;
		// raw_blobs[BLOB1].x[_X_] = ball_state.x[_X_];
		// raw_blobs[BLOB1].x[_Y_] = ball_state.x[_Y_];
		// raw_blobs[BLOB1].x[_Z_] = ball_state.x[_Z_];
		// last_frame_counter = frame_counter;
		// send_raw_blobs();
	// }
	
	beerpong_environmentVar env = {
		.ballX = ball_state.x[_X_],
		.ballY = ball_state.x[_Y_],
		.ballZ = ball_state.x[_Z_],
		.cupX = cup_state.x[_X_],
		.cupY = cup_state.x[_Y_],
		.cupZ = cup_state.x[_Z_],
		.varOrient = orientBack
	};
	
	sendUserGraphics("beerpong",&(env), sizeof(beerpong_environmentVar));
}
