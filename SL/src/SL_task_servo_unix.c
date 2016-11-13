/*!=============================================================================
  ==============================================================================

  \ingroup SLtask
  
  \file    SL_task_servo_unix.c

  \author  Stefan Schaal
  \date    1999

  ==============================================================================
  \remarks

  main program for the task servo

  ============================================================================*/

#define _GNU_SOURCE
// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_task_servo.h"
#include "SL_tasks.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_man.h"
#include "SL_dynamics.h"
#include "SL_objects_defines.h"
#include <sys/mman.h>
#include <sched.h>
#include <sys/utsname.h>
#include <string.h>

#define TIME_OUT_NS  1000000

// global variabes
char initial_user_command[100]="";

// global functions 
void task_servo(void);

// local functions

// external functions
extern void initUserTasks(void);

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

  // initializes the servo
  init_task_servo();
  read_whichDOFs(config_files[WHICHDOFS],"task_servo");

  // generic computations
  init_user_task();

  // reset task_servo variables
  servo_enabled           = 1;
  task_servo_calls        = 0;
  task_servo_time         = 0;
  last_task_servo_time    = 0;
  task_servo_errors       = 0;
  task_servo_rate         = servo_base_rate/(double) task_servo_ratio;

  setTaskByName(NO_TASK);
  changeCollectFreq(task_servo_rate);
  
  // the user tasks as defined in initUserTasks.c 
  initUserTasks();

  // spawn command line interface thread
  spawnCommandLineThread(initial_user_command);


  // get the servo parameters and set scheduler accordingly
  char name[100];
  int delay_ns = FALSE;
  int servo_priority = 25;
  int servo_stack_size = 2000000;
#ifndef __APPLE__  
  int cpuID = sizeof(cpu_set_t)-2;
  if(cpuID < 0)
	  cpuID = 0;
#else
  int cpuID = 0;
#endif

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
	  }

	  struct sched_param sParam;
	  sParam.sched_priority = servo_priority;

	  if ( sched_setscheduler(0, SCHED_FIFO, &sParam) ) {
		  printf("set sched FAILED\n");
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


  // reset the simulation
  if (!real_robot_flag)
    reset();
  
  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);

  // run the servo loop
  while (servo_enabled) {

    // wait to take semaphore 
    if (semTake(sm_task_servo_sem,WAIT_FOREVER) == ERROR)
      stop("semTake Time Out -- Servo Terminated");

    // lock out the keyboard interaction 
    sl_rt_mutex_lock( &mutex1 );

    // run the task servo routines
    if (!run_task_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

  printf("Task Servo Error Count = %d\n",task_servo_errors);

  return TRUE;

}
 
