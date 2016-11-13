#include "OptiTrackParser.h"

// SL general includes of system headers
#include "SL_system_headers.h"

/* user specific headers */
#include "SL.h"
#include "SL_vision_servo.h"

#include <iostream>

extern "C" {

static OptiTrackParser* opt = 0;

int init_vision_hardware(void) {

    char ip[300] = "127.0.0.1";
    int port = 1511;

    get_string("Set the ip of the optitrack:", ip, ip);
    get_int("Set the optitrack port:", port, &port);

    opt = new OptiTrackParser(ip, (uint32_t) port);

    return TRUE;
}


/* external functions */
int acquire_blobs(Blob2D [][2 + 1]) {


    for (int i =1; i <= max_blobs; ++i ) {
        raw_blobs[i].status = 0;
        blobs[i].status = 0;
    }

    OptiTrackParser::Frame frm;
    if ( opt->processNextMessageSL(frm) ) {

        for ( int j = 0; j < frm.rigidBodies.size(); ++j ) {

            raw_blobs[j+1].status = 1;

            raw_blobs[j+1].x[_X_] = frm.rigidBodies[j].x;
            raw_blobs[j+1].x[_Y_] = frm.rigidBodies[j].y;
            raw_blobs[j+1].x[_Z_] = frm.rigidBodies[j].z;

            raw_blobs[j+1].o[_Q0_] = frm.rigidBodies[j].qw;
            raw_blobs[j+1].o[_Q1_] = frm.rigidBodies[j].qx;
            raw_blobs[j+1].o[_Q2_] = frm.rigidBodies[j].qy;
            raw_blobs[j+1].o[_Q3_] = frm.rigidBodies[j].qz;
        }

    }

    struct timespec cTime;
    static struct timespec prevTime;
    clock_gettime(CLOCK_MONOTONIC, &cTime);


    for (int i =1; i <= max_blobs; ++i ) {
        if ( raw_blobs[i].status ) {
            raw_blobs[i].timestamp =  (double) cTime.tv_sec+ (double) (cTime.tv_nsec) / 1e9;
            raw_blobs[i].time_diff = ((double)(cTime.tv_sec - prevTime.tv_sec))+((double)(cTime.tv_nsec - prevTime.tv_nsec))/1e9;
        }
    }

    count_all_frames++;
    prevTime = cTime;


    return TRUE;
}

int init_user_vision(void) {
    return TRUE;
}

};
