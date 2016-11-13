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
#include "SL_vision_servo.h"


int init_vision_hardware(void) {
	return FALSE;
}

int acquire_blobs(Blob2D blobs[][2+1]) {
	return FALSE;
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

  return TRUE;
}


