/*!=============================================================================
  ==============================================================================

  \file    SL_user_vision.c

  \author  Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks
  
  Allows user specific vision servo initializations
  
  ============================================================================*/
  
// SL general includes of system headers
#include "SL_system_headers.h"

/* user specific headers */
#include "SL.h"
#include "SL_user.h"
#include "SL_man.h"
#include "SL_vision_servo.h"

#include "RoLL_vision_comm.h"


typedef struct Blob2DInfo {
  int status;
  double x;
  double y;
  double area;
  double left;
  double top;
  double right;
  double bottom;
  double aspect_ratio;
  double orient;
} Blob2DInfo;

typedef struct Frame {
  int counter;  /* the frame counter */
  Blob2DInfo blobinfo[MAX_BLOBS+1][N_CAMERAS+1];
} Frame;


/* global variables */
int camera_assignment[CAMERA_2+1][MAX_BLOBS+1] = {
  {MAX_BLOBS,0,0,0,0,0,0},
  {MAX_BLOBS, PAIR1_LEFT_CAMERA,  PAIR2_RIGHT_CAMERA, PAIR2_LEFT_CAMERA,  PAIR1_RIGHT_CAMERA, PAIR1_LEFT_CAMERA, PAIR1_RIGHT_CAMERA},  
  {MAX_BLOBS, PAIR1_RIGHT_CAMERA, PAIR1_LEFT_CAMERA,  PAIR2_RIGHT_CAMERA, PAIR2_LEFT_CAMERA,  PAIR2_LEFT_CAMERA, PAIR2_RIGHT_CAMERA}
}; 
int relabel[4+1] = {4,  1,   4,   2,  3};

static Frame           the_frame;
static int             last_counter;
static int             frame_counter;
static struct timespec nsleep;

static tPackage inPackage, lastPackage;
static int cycle_time = DEFAULT_CYCLE, port_no = DEFAULT_PORT ;
static char ip[150];

int ball_present = 0, no_ball_present = 0;
double min_conf[4+1], max_conf[4+1]; 
int min_counter, max_counter; 
int ballInPicture[4+1];

/* local variables */
extern int stereo_mode;

  
/* local functions */

/* external functions */
void blob_stats();
void comm_stats();
void reloadCameraParamaters(void);
void estimateConfidences();

void init_hardware(){
  int i;

  if(init_vision_hardware()) {
    no_hardware_flag = FALSE;
    printf("The real vision system has successfully started!\n");
  } else {
    no_hardware_flag = TRUE;
    printf("The real vision system has failed to start!\n");
  }
};

int loadConfidences(char *filename) {
    int  i, j;
    FILE *fp;
    char string[100];

    /* read in the blob confidences file */
    sprintf (string, "%s%s", CONFIG,filename);
    fp = fopen (string, "r");

    if (fp == NULL) {
	printf("Cannot find the blob confidences file >%s<\n",string);
	beep(1); 
	return FALSE;
    }

    printf("Reading blob confidences >%s< ...", filename);
    for(i=1; i<=4; i++)
	fscanf (fp, "%lf", & min_conf[i]);
    for(i=1; i<=4; i++)
	fscanf (fp, "%lf", & max_conf[i]);    
    fclose(fp);
    printf(" done\n");

    return TRUE;
}

int saveConfidences(char *filename) {
    int  i, j;
    FILE *fp;
    char string[100];

    /* read in the blob confidences file */
    sprintf (string, "%s%s", CONFIG,filename);
    fp = fopen (string, "w");

    if (fp == NULL) {
	printf("Cannot find the blob confidences file >%s<\n",string);
	beep(1); 
	return FALSE;
    }

    printf("Writing blob confidences ...");
    for(i=1; i<=4; i++)
	fprintf (fp, "%lf  ", min_conf[i]);
    fprintf (fp, "\n");
    for(i=1; i<=4; i++)
	fprintf (fp, "%lf  ", max_conf[i]);    
    fprintf (fp, "\n");
    fclose(fp);
    printf(" done\n");

    return TRUE;
}

void estimateConfidences() {
 int ans = 1, i;

 // Estimate minimum confidences
 printf("Make sure no ball is in the picture...\n");
 get_int("Press 1 to go on", ans, &ans);
 if(!ans) return;
 min_counter = 0;
 for(i=1; i<=4; i++)
    min_conf[i] = 0;
 no_ball_present = 1;
 get_int("Press any number to stop", ans, &ans);
 no_ball_present = 0;  
 for(i=1; i<=4; i++)
    min_conf[i] /= (double)min_counter;

 // Estimate maximum confidences
 printf("Make sure the ball is in the picture!\n");
 get_int("Press 1 to go on", ans, &ans);
 if(!ans) return;
 max_counter = 0;
 for(i=1; i<=4; i++)
    max_conf[i] = 0;
 ball_present = 1;
 get_int("Press any number to stop", ans, &ans);
 ball_present = 0;
 for(i=1; i<=4; i++)
    max_conf[i] /= (double)max_counter;

 saveConfidences("BlobConfidences.cf");
}




/*****************************************************************************
******************************************************************************
Function Name	: init_vision_hardware
Date		: June 2008
   
Remarks:

        initializes communication with the vision system

******************************************************************************
Paramters:  (i/o = input/output)

     none

*****************************************************************************/
int
init_vision_hardware(void)
{
  int realVision = FALSE;

  /* init the vision server communication */
  get_int("Use the real vision servo?",1,&realVision);
  if(realVision) {
     printf("Vision Hardware being initialized ...\n");
     show_devices();
     strcpy(ip, DEFAULT_IP);	
     if(receiver_start(ip, port_no, cycle_time)) {
       perror("Server could not open...");
     } else {
       printf("Server opened...\n");
     }
     loadConfidences("BlobConfidences.cf");

     /* the sleep structure */
     nsleep.tv_sec  = 0;
     nsleep.tv_nsec = 100;
     return TRUE;
  } else {
     return FALSE;
  };
}


/*!*****************************************************************************
 *******************************************************************************
 \note  acquire_blobs
 \date  July 2008
 
 \remarks 
 
read the current information about the blobs
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  [out] blobs : array of blob structures

 ******************************************************************************/
int
acquire_blobs(Blob2D blobs[][2+1])

{
  int   i,j, c, rc;
  char  buffer[2];
  
  /* reset all the blobs to be non existent */

  for (i = 1; i<=MAX_BLOBS; ++i) {
    the_frame.blobinfo[i][CAMERA_1].status = FALSE;
    the_frame.blobinfo[i][CAMERA_2].status = FALSE;
  }

   /* now we are at the beginning of a frame */
  last_counter = the_frame.counter;
  
   /* read the entire frame */
  getPackage(&inPackage);
  
  for(i=1; i<=4; i++)
    if(isnan(inPackage.conf[i-1])) inPackage.conf[i-1] = 0;

  if(no_ball_present) {
    for(i=1; i<=4; i++)
    	min_conf[i] += inPackage.conf[i-1];
    min_counter++;
  } else if(ball_present) {
    for(i=1; i<=4; i++)
    	max_conf[i] += inPackage.conf[i-1];
    max_counter++;
  }

  for(i=1; i<=4; i++)
  	ballInPicture[i] = (inPackage.conf[i-1] - min_conf[i]) > 
		(0.25 * ( max_conf[i] - min_conf[i]));


  for(j=1; j<=MAX_BLOBS; j++) for(i=CAMERA_1; i<=CAMERA_2; i++) {
    the_frame.blobinfo[j][i].x = inPackage.x[relabel[camera_assignment[i][j]]-1];
    the_frame.blobinfo[j][i].y = inPackage.y[relabel[camera_assignment[i][j]]-1];
    the_frame.blobinfo[j][i].status =  ballInPicture[relabel[camera_assignment[i][j]]];
    
    if(isnan(the_frame.blobinfo[j][i].x)||isnan(the_frame.blobinfo[j][i].x)) {
      printf("NaN - Blob %d Cam %d", j, i); fflush(stdout);
      the_frame.blobinfo[j][i].status = FALSE;
    }
  }
  if(inPackage.index>lastPackage.index)
    count_all_frames++;
  lastPackage = inPackage;

  // copy the data into the global structures 
  for (i=1; i<=MAX_BLOBS; ++i) {
    for(c=CAMERA_1; c<=CAMERA_2; ++c) {
      blobs[i][c].status = the_frame.blobinfo[i][c].status;
      if(blobs[i][c].status) {
         blobs[i][c].x[_X_] = the_frame.blobinfo[i][c].x;
         blobs[i][c].x[_Y_] = the_frame.blobinfo[i][c].y;
      };
    };
  }

  return TRUE;
}  

/*!*****************************************************************************
 *******************************************************************************
 \note  init_user_vision
 \date  July 1998
 
 \remarks 
 
 initializes user specific vision functionality
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none 

 ******************************************************************************/
int
init_user_vision(void)

{
  int i,j,n;

  // Add includes of our vision system
  addToMan("blob","displays status information about blob",blob_stats);
  addToMan("comm","displays status information about communication",comm_stats);
  addToMan("reload","reloads the camera parameters", reloadCameraParamaters);
  addToMan("conf","restimates the confidences", estimateConfidences);
  addToMan("iv","turns on the real vision servo", init_hardware);

  printf("\n\nIMPORTANT: To enable the real vision type 'iv' !!!\n\n");

  stereo_mode=VISION_2D_MODE;

  return TRUE;
}


