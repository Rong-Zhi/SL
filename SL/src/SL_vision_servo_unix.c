/*!=============================================================================
  ==============================================================================

  \ingroup SLvision


  \file    SL_vision_servo_unix.c

  \author  Stefan Schaal
  \date    Nov. 2007

  ==============================================================================
  \remarks

  Initialization and semaphore communication for the vision servo
  under unix

  ============================================================================*/

#define _GNU_SOURCE
// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_common.h"
#include "SL_shared_memory.h"
#include "SL_unix_common.h"
#include "SL_collect_data.h"
#include "SL_man.h"
#include "SL_vision_servo.h"
#include <sys/mman.h>
#include <sched.h>
#include <sys/utsname.h>
#include <string.h>

#define TIME_OUT_NS  100000

/* global variables */
extern int stereo_mode;

/* local variables */

/* global functions */

/* local functions */


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
  int rc;

  // parse command line options
  parseOptions(argc, argv);

  // adjust settings if SL runs for a real robot
  setRealRobotOptions();

  // signal handlers
  installSignalHandlers();

  // initializes the servo
  init_vision_servo();

  // setup servo
  servo_enabled           = 1;
  vision_servo_calls      = 0;
  last_vision_servo_time  = 0;
  vision_servo_time       = 0;
  servo_time              = 0;
  vision_servo_errors     = 0;

  changeCollectFreq(vision_servo_rate);

  // initialize all vision variables to safe values
  init_vision_states();

  // spawn command line interface thread
  spawnCommandLineThread(NULL);

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

  // signal that this process is initialized
  semGive(sm_init_process_ready_sem);

  // run the servo loop
  while (servo_enabled) {

    // check whether there is some hardware interaction -- the vision hardware would
    // provide the clocking
    if (!no_hardware_flag && !raw_blob_overwrite_flag) {

      // this is just added to notice when SL is aborted
      if (semGet(sm_vision_servo_sem,&rc) == ERROR)
	stop("semTake Time Out -- Servo Terminated");

      if (!acquire_blobs(raw_blobs2D)) {
	no_hardware_flag = TRUE;
      }

    } else { // with no hardware, we rely on the internal clock

      // wait to take semaphore 
      if (semTake(sm_vision_servo_sem,WAIT_FOREVER) == ERROR)
	stop("semTake Time Out -- Servo Terminated");
      
      // reset the blob status
      for (i=1; i<=max_blobs; ++i) {
	raw_blobs2D[i][1].status = FALSE;
	raw_blobs2D[i][2].status = FALSE;
      }
      
    }

    // lock out the keyboard interaction 
    sl_rt_mutex_lock( &mutex1 );

    // run the task servo routines
    if (!run_vision_servo())
      break;

    // continue keyboard interaction
    sl_rt_mutex_unlock( &mutex1 );

  }  /* end servo while loop */

  printf("Vision Servo Error Count = %d\n",vision_servo_errors);

  return TRUE;

}

