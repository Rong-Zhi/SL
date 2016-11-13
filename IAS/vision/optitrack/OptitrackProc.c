// SL general includes of system headers
#include "SL_system_headers.h"

/* private includes */
#include "utility.h"
#include "SL.h"
#include "SL_common.h"
#include "SL_vision_servo.h"
#include "nrutil.h"

int init_vision_processing(void) {
    return TRUE;
}

int init_pp(char *name)

{
    return TRUE;
}

void process_blobs(Blob2D not_used[][2+1])  {

    int i,j,n;
    static double old_blob[N_CART+N_QUAT+1];
    static int firsttime = 1;
    if ( firsttime ) {
        firsttime  = 0;
        for (  j= _X_;  j<= _Z_;  ++j )
            old_blob[j] = raw_blobs[1].x[j];
        for (  j= _Q0_;  j<= _Q3_;  ++j )
            old_blob[j+_Z_] = raw_blobs[1].o[j];
    }

    for (i=1; i<=max_blobs; ++i) {

        if (raw_blobs[i].status) {

            for (  j= _X_;  j<= _Z_;  ++j ) {
                blobs[i].blob.x[j]   = raw_blobs[i].x[j];
                blobs[i].blob.xd[j]  = ( raw_blobs[i].x[j] -  old_blob[j] ) / raw_blobs[i].time_diff;
                blobs[i].blob.xdd[j] = 0.0;
                old_blob[j] = raw_blobs[i].x[j];
            }

            for (  j= _Q0_;  j<= _Q3_;  ++j ) {
                blobs[i].blob_or.q[j]   = raw_blobs[i].o[j];
                blobs[i].blob_or.qd[j]  = ( raw_blobs[i].o[j] -  old_blob[j+_Z_] ) / raw_blobs[i].time_diff;
                blobs[i].blob_or.qdd[j] = 0.0;
                old_blob[j+_Z_] = raw_blobs[i].o[j];
            }
            blobs[i].status   = TRUE;
        }
        else
            blobs[i].status   = FALSE;
    }

    return;

}

/*!*****************************************************************************
 *******************************************************************************
\note  init_vision_states
\date  August 7, 1995

\remarks

initializes all states of the filters to safe initial values

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void init_vision_states(void) {
    int i,j;

    /* initialize the vision variables */
    bzero((char *) blobs, (max_blobs+1)*sizeof(SL_VisionBlob));
    bzero((char *) raw_blobs, (max_blobs+1)*sizeof(Blob3D));
}

