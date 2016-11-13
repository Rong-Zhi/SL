#include "ias_utilities.h"
#include "ias_common.h"

#include <math.h>


int firstGotoPos = 1;
//double sampling_time = 0.002;


void cp_sl_states(SL_DJstate* src, SL_DJstate* target)
{
	int i;

	for (i = 1; i <= n_dofs; i++)
	{
		target[i].th   = src[i].th;
		target[i].thd  = src[i].thd;
		target[i].thdd = src[i].thdd;
		target[i].uff  = src[i].uff;
		target[i].uex  = src[i].uex;
	}
}


void cp_state_vec(SL_Cstate state, Vector x)
{
	int i;

	for(i = 1; i <= N_CART; i++)
	{
		x[i]   = state.x[i];
		x[i+3] = state.xd[i];
	}

	x[7] = 1;
}

void cp_vec_state(SL_Cstate *state, Vector x)
{
	int i;

	for(i = 1; i <= N_CART; i++)
	{
		(*state).x[i]  = x[i];
		(*state).xd[i] = x[i+3];
	}
}


/*****************************************************************************
 ******************************************************************************
 Function Name	: read_Jinv_weights
 Date		: June 2005
 Remarks:
 
 parses Jinv weights from the file
 
 ******************************************************************************
 Paramters:  (i/o = input/output)
 
 none
	 
 *****************************************************************************/
Vector getJinvWeights() 
{
	int j, i;
	char  string[100];
	FILE *in;
	int read = TRUE;
	static Vector Jinv_w;
	static int first_time = 1;

	if (first_time)
	{
		Jinv_w = my_vector(1, n_dofs);
		/* get the max, min, and offsets of the position sensors */
		
		sprintf(string,"%s%s",PREFS,JINV_WEIGHTS_FILE);
		in = fopen(string,"r");
		if (in == NULL) {
			printf("ERROR: Cannot open file >%s<!\n",string);
			read = FALSE;
		}
	
		/* find all joint variables and read them into the appropriate array */
	
		for (i = 1; i <= n_dofs; ++i) 
		{
			if (!find_keyword(in, &(joint_names[i][0]))) 
			{
				printf("ERROR: Cannot find JInv for >%s<!\n",joint_names[i]);
				fclose(in);
				read = FALSE;
			}
			if ( fscanf(in,"%lf",&Jinv_w[i]) != 1 )
				read = FALSE;
		}
		fclose(in);

		if (read == FALSE)
		{
			for (i = 1; i <= n_dofs; ++i) 
			{
				Jinv_w[i] = 1.0;
			}			
		}
		first_time = 0;
	}
	
	return Jinv_w;	
}

int check_jointLimits(SL_DJstate *des, double th)
{
	int i;
	int flag = TRUE;

	for (i = 1; i <= n_dofs; i++) 
	{
		if (des[i].th + th > joint_range[i][MAX_THETA]) 
		{
		      flag = FALSE;
		}
		if (des[i].th - th < joint_range[i][MIN_THETA]) 
		{
		      flag = FALSE;
		}
		DEBUGPRINT("%f: %f %f, ",des[i].th, joint_range[i][MIN_THETA], joint_range[i][MAX_THETA]);
    }
    	
 	return flag;
}


int set_toJointLimits(SL_DJstate *des, double th)
{
	int i;
	int flag = TRUE;

	for (i = 1; i <= n_dofs; i++) 
	{
		if (des[i].th + th > joint_range[i][MAX_THETA]) 
		{
			des[i].th = joint_range[i][MAX_THETA] - th;
		    flag = FALSE;
		}
		if (des[i].th - th < joint_range[i][MIN_THETA]) 
		{
			des[i].th = joint_range[i][MIN_THETA] + th;
			
		    flag = FALSE;
		}
    }
    	
 	return flag;

}
