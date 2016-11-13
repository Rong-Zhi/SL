/*!=============================================================================
  ==============================================================================

  \ingroup SLsimulation

  \file    SL_simulation_servo_unix.c

  \author  Stefan Schaal
  \date    2007

  ==============================================================================
  \remarks

  main program for running a numerical simulation of a robot, i.e.,
  this is where the equations of motion are integrated.

  ============================================================================*/

#define _GNU_SOURCE
// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_integrate.h"
#include "SL_objects.h"
#include "SL_terrains.h"
#include "SL_simulation_servo.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_man.h"
#include <sys/mman.h>
#include <sched.h>
#include <sys/utsname.h>
#include <string.h>

#define TIME_OUT_NS  1000000

// global variables

// local variables
static int pause_flag = FALSE;


// global functions 

// local functions

// external functions

/*!*****************************************************************************
*******************************************************************************
\note  main
\date  Feb 1999
\remarks 

initializes everything and starts the servo loop

*******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     argc : number of elements in argv
\param[in]     argv : array of argc character strings

******************************************************************************/
int 
main(int argc, char**argv)
{
  int i, j;

  // parse command line options
  parseOptions(argc, argv);

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initalize the servo
  if (!init_simulation_servo())
    return FALSE;

  // spawn command line interface thread
  spawnCommandLineThread(NULL);


  // get the servo parameters and set scheduler accordingly
  char name[100];
  int delay_ns = FALSE;
  int servo_priority = 90;
  int servo_stack_size = 2000000;
  int cpuID = 0;

  sprintf(name,"%s_servo",servo_name);
  if(!read_servoParameters(config_files[SERVOPARAMETERS],name,&servo_priority, 
			  &servo_stack_size,&cpuID,&delay_ns))
  {
	  printf("servo parameters couldn't be set");
  }
  
  if(servo_priority > 0){
	  int rt_fail = 0;
	  if ( mlockall(MCL_CURRENT | MCL_FUTURE) ) {
		  printf("mlockall FAILED\n");
		  rt_fail = 1;
	  }
#ifndef __APPLE__
	  cpu_set_t mask;
	  CPU_ZERO(&mask);
	  CPU_SET(cpuID, &mask);
	  if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) < 0) {
		  printf("set sched FAILED\n");
		  rt_fail = 1;
	  }

	  struct sched_param sParam;
	  sParam.sched_priority = servo_priority;

	  if ( sched_setscheduler(0, SCHED_FIFO, &sParam) ) {
		  printf("set sched FAILED\n");
		  rt_fail = 1;
	  }
#endif

	  struct utsname unameData;
	  uname(&unameData);
	  if(strstr(unameData.release, "rt") == NULL) {
		  printf("Kernel is not realtime\n");
	  }
	  if(rt_fail){
		  printf("Fix permissions for realtime in /etc/security/limits.conf\n");
	  }
  }

  // boardcast the current state such that the motor servo can generate a command
  send_sim_state();
  send_misc_sensors();
  send_contacts();
  //semGive(sm_motor_servo_sem);  

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);
  
  // run the servo loop
  while (servo_enabled) {

    // check for PAUSE
    if (semTake(sm_pause_sem,NO_WAIT) != ERROR) {
      if (pause_flag)
	pause_flag = FALSE;
      else
	pause_flag = TRUE;
    }

    if (pause_flag) {
      usleep(10000);
      continue;
    }

    // wait to take semaphore 
    //if (semTake(sm_simulation_servo_sem,WAIT_FOREVER) == ERROR) {
    //  printf("semTake Time Out -- Servo Terminated\n");
    //  return FALSE;
    //}

    // lock out the keyboard interaction 
    sl_rt_mutex_lock( &mutex1 );

    // run the simulation servo routines
    if (!run_simulation_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

  return TRUE;

}
 
