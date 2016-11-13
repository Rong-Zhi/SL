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

#include "ballonbeam.h"
#include "ballonbeam_env.h"

static double den, nom, fact, rati;
static double k = 200;	// according to eqs this is k/(2*m)
static double d = 1.5;	// according to eqs this is d/m

SL_Cstate beamState;
SL_Cstate ballState;
static SL_quat beamOrient;
static SL_quat beamRot;

int addBallOnBeamVars ()
{
	int i;
	char varname[30];
	for (i = _X_; i <= _Z_; i++)
	{				// Sim-Ball-Positions
		sprintf (varname, "x_ball_sim_%d", i);
		addVarToCollect ((char *) &(ballState.x[i]), varname, "m", DOUBLE,FALSE);
	};
	for (i = _X_; i <= _Z_; i++)
	{				// Sim-Ball_Velocities
		sprintf (varname, "dx_ball_sim_%d", i);
		addVarToCollect ((char *) &(ballState.xd[i]), varname, "m/s", DOUBLE,FALSE);
	};
	for (i = _X_; i <= _Z_; i++)
	{				// Sim-Ball-Accelerations
		sprintf (varname, "ddx_ball_sim_%d", i);
		addVarToCollect ((char *) &(ballState.xdd[i]), varname, "m/s^2", DOUBLE, FALSE);
	};
	
	return TRUE;
}


void resetBallOnBeamSimulation()
{
	double beamDist = (beamHeight / 2.f) + ballRadius;
    
	ballState.x[_X_] = beamState.x[_X_];
	ballState.x[_Y_] = beamState.x[_Y_];
	ballState.x[_Z_] = beamState.x[_Z_] + beamDist;
        
	ballState.xd[_X_] = 0;
	ballState.xd[_Y_] = 0;
	ballState.xd[_Z_] = 0;
}


int simBallOnBeamTask()
{
	int j, i, dim;
	double eps, deps, ball_rxy;
	double vec[4],  rvec[4], dvec[4], epsv[4], depsv[4];
	static SL_quat orientBack, orientForward;
	double diff_x[4], diff_xd[4];
    
	double beamDist = (beamHeight / 2.f) + ballRadius;
    
	static int firsttime = TRUE;
	
	if(firsttime)
	{
		firsttime = FALSE;
		setDefaultEndeffector();
		endeff[RIGHT_HAND].x[_Z_] = 0.2;
	}
	
	for (dim = _X_; dim <= _Z_; dim++)
	{
		beamState.x[dim]    = cart_state[RIGHT_HAND].x[dim];
		beamState.xd[dim]   = cart_state[RIGHT_HAND].xd[dim];
		beamState.xdd[dim]  = cart_state[RIGHT_HAND].xdd[dim];
	}
	
	for (dim = _Q0_; dim <= _Q3_; dim++)
	{
		beamOrient.q[dim] = cart_orient[RIGHT_HAND].q[dim];
	}
    
	beamRot.q[_Q0_] = cos(PI/4);
	beamRot.q[_Q1_] = -sin(PI/4);
	beamRot.q[_Q2_] = 0;
	beamRot.q[_Q3_] = 0;
	
	mult_quat(&beamOrient, &beamRot, &beamOrient); 

	ballState.xdd[_X_] = 0;
	ballState.xdd[_Y_] = 0;
	ballState.xdd[_Z_] = 0;

	// quaternions for hand rotation
	for (dim = _Q0_; dim <= _Q3_; dim++)
	{
		orientForward.q[dim]    = beamOrient.q[dim];
		orientBack.q[dim]       = beamOrient.q[dim];
	}
    
	orientForward.q[1] = cos(PI-acos(beamOrient.q[1]));
    
	// compute distance and velocity of ball relative to the center of the beam
	for(dim = _X_; dim <= _Z_; dim++) {
		epsv[dim]  = beamState.x[dim] - ballState.x[dim];
		depsv[dim] = beamState.xd[dim] - ballState.xd[dim]; 
	};   
    
    for (i = 0; i < 3; ++i)
    {
        vec[i]  = epsv[i];
        dvec[i] = depsv[i];
    }
	rotate_quat(orientForward, epsv, vec);
	rotate_quat(orientForward, depsv, dvec);  
	
	if(( fabs(vec[1]) < beamLength / 2.f ) && 
       ( fabs(vec[2]) < beamWidth / 2.f ) &&
	   ( fabs(vec[3]) < beamDist )  )
	{
		eps     = vec[3];
		deps    = dvec[3];
        
        if( (eps <= beamDist) && (eps>=0) ) 
        { 
            dvec[3] = (restitution) * fabs(task_servo_rate * deps);
            vec[3]  = (beamDist);
        } 
        else if( (eps >- beamDist) && (eps < 0) ) 
        { 
            dvec[3] = -(restitution) * fabs(task_servo_rate * deps);
            vec[3]  = -(beamDist);
        } 
        else 
        {
            printf(" -- ERROR in ballOnBeam.c\n");
            fflush(stdout);
        }
        
        rotate_quat(orientBack, vec, epsv);  
        rotate_quat(orientBack, dvec, depsv);  
        
        
        for(dim = _X_; dim <= _Z_; dim++) 
        {
            ballState.xdd[dim] = -depsv[dim]; 
            ballState.x[dim]   = beamState.x[dim] - epsv[dim];            
        }
 	}
    
	ballState.xd[_Y_]   = ballState.xdd[_Y_] = 0.f; 
	ballState.xdd[_Z_] -= gravity ;		

	// integrate over time
	for(dim = _X_; dim <= _Z_; dim++) 
	{
		ballState.xd[dim] += ballState.xdd[dim] / task_servo_rate;
		ballState.x[dim]  += ballState.xd[dim] / task_servo_rate;
		
	}
	return TRUE;
}

void sendBallOnBeamGraphics()
{
	static int last_frame_counter = -999;
	
	if (last_frame_counter != frame_counter) {
		// in this way you can read the ball position from the vision system 
		raw_blobs[BLOB1].status = TRUE;
		raw_blobs[BLOB1].x[_X_] = ballState.x[_X_];
		raw_blobs[BLOB1].x[_Y_] = ballState.x[_Y_];
		raw_blobs[BLOB1].x[_Z_] = ballState.x[_Z_];
		send_raw_blobs(); // generate the ball using the vision system
		last_frame_counter = frame_counter;
	}
	
	// environment variables used to display the 3D animation
	ballonbeam_environmentVar env = {
		.ballX = blobs[1].blob.x[_X_], // here we are using also the position of the ball (sending to the graphics file)
		.ballY = blobs[1].blob.x[_Y_], // to draw another ball (above the previous one).
		.ballZ = blobs[1].blob.x[_Z_], // This is just another way to draw the ball (but the more you know, the better)
		.beamX = beamState.x[_X_],
		.beamY = beamState.x[_Y_],
		.beamZ = beamState.x[_Z_],
		.varOrient = beamOrient
	};

	sendUserGraphics("ballOnBeam",&(env), sizeof(ballonbeam_environmentVar));
}
