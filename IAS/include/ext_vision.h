#ifndef _EXT_VISION_H_
#define _EXT_VISION_H_ 1

//#include "SL_user.h"
#include "sharedmemory.h"
#include "utility.h"
#include "SL.h"


enum {
	ext_vision_obj_num = 7,
	ext_vision_obj_size = sizeof(double),
	ext_vision_buffer_size = ext_vision_obj_num * ext_vision_obj_size,
	ext_vision_sigSem_id = 58045,
	ext_vision_sem_id = 55025,
	ext_vision_shm_id = 100003
};


typedef struct ext_vision_comm_s {

	semaphore semSig;

	semaphore semVision;
	sharedmemory shmVision;

	double buffer[ext_vision_obj_num];
	unsigned long bufferSize;

} ext_vision_comm_s;


extern int ext_vision_comm_init ( ext_vision_comm_s* ctx );

/** Return 1 on success and 0 otherwise
 *
 */
extern int ext_vision_comm_readNewDataNb ( ext_vision_comm_s* ctx, SL_Cstate* vData );
extern int ext_vision_comm_readNewDataBlk ( ext_vision_comm_s* ctx, SL_Cstate* vData );






typedef struct ext_vision_calib_s {

	Matrix observ_m;
	Matrix robot_m;
	unsigned long observ_size; //rows with data
	unsigned long observ_max_size;

	Matrix transf;
	int isTransfValid;

} ext_vision_calib_s;


extern void ext_vision_calib_init ( ext_vision_calib_s* ctx );
extern void ext_vision_calib_reset ( ext_vision_calib_s* ctx, int realloc );
extern void ext_vision_calib_setMax ( ext_vision_calib_s* ctx, unsigned long newSize, int keepObs );

extern int ext_vision_calib_addPoint ( ext_vision_calib_s* ctx, SL_Cstate* vData, SL_Cstate* rData );

extern int ext_vision_calib_calcTrans ( ext_vision_calib_s* ctx );

extern int ext_vision_calib_transPoint ( ext_vision_calib_s* ctx, SL_Cstate* in, SL_Cstate* out );


extern void ext_vision_calib_printTransMatrix ( ext_vision_calib_s* ctx );
extern int ext_vision_calib_exportTransMatrix ( ext_vision_calib_s* ctx, char* filename );
extern int ext_vision_calib_importTransMatrix ( ext_vision_calib_s* ctx, char* filename );


#endif // _EXT_VISION_H_
