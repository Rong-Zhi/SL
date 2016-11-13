/*!=============================================================================
  ==============================================================================

  \file    SL_user_commands.c

  \author 
  \date   

  ==============================================================================
  \remarks

      various functions that allow the user to monitor the robot

  ============================================================================*/

// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "SL_motor_servo.h"
#include "utility.h"
#include "SL_man.h"

/* global variables */

/* variables for the motor servo */

/* global functions */
int init_user_commands(void);
void set_special_endeffector(double x, double y, double z);

/* local functions */


/*!*****************************************************************************
 *******************************************************************************
\note  set_special_endeffector
\date  Nov 2010
\remarks 

 initializes the endeffector to a different length

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/

void 
set_special_endeffector(double x, double y, double z) {

  int i;

  for (i=1; i<=N_ENDEFFS; ++i) {
    endeff[i].m       = 0.0;
    endeff[i].mcm[_X_]= 0.0;
    endeff[i].mcm[_Y_]= 0.0;
    endeff[i].mcm[_Z_]= 0.0;
    endeff[i].x[_X_]  = x;
    endeff[i].x[_Y_]  = y;
    endeff[i].x[_Z_]  = z;
    endeff[i].a[_A_]  = 0.0;
    endeff[i].a[_B_]  = 0.0; //-PI/2.;
    endeff[i].a[_G_]  = 0.0;
  }

}



/*!*****************************************************************************
 *******************************************************************************
\note  init_user_commands
\date  Feb 1999
\remarks 

 initializes various things

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int
init_user_commands(void)
{
  /* addToMan("where","print all current state information",where); */
  return TRUE;
}

