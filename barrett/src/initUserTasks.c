/*============================================================================
==============================================================================
                      
                              initUserTasks.c
 
==============================================================================
Remarks:

         Functions needed to initialize and link user tasks for the
         simulation

============================================================================*/

#include "SL.h"
#include "SL_user.h"
#include "SL_man.h"
#include "SL_task_servo.h"

/* global variables */

/* local variables */
static int user_tasks_initialized = FALSE;

/*****************************************************************************
******************************************************************************
Function Name	: initUserTasks
Date		: June 1999
   
Remarks:

      initialize tasks that are not permanently linked in the simulation
      This replaces the <ltask facility in vxworks -- just that we cannot
      do on-line linking in C.

******************************************************************************
Paramters:  (i/o = input/output)

  none   

*****************************************************************************/
void initUserTasks(void)
{

#ifdef _CMAKE_SL_TASK_AUTO_ADD_
#include <user_tasks_autodetect.h>
#endif // _CMAKE_SL_TASK_AUTO_ADD_

	sprintf(initial_user_command,"go0");
	
}

