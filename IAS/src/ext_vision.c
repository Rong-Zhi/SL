#include "ext_vision.h"


#include <string.h>
#include <stdio.h>

#include "ias_matrix_utilities.h"


int ext_vision_comm_init ( ext_vision_comm_s* ctx ) {

	ctx->bufferSize = ext_vision_buffer_size;

	memset( ctx->buffer, 0, ctx->bufferSize );

	initSemaphore( &(ctx->semSig) );
	setTimeOut( &(ctx->semSig), 1200);
	int ok = attachSEM( &(ctx->semSig), ext_vision_sigSem_id );

	if ( ok > 0 ) {
		initSemaphore( &(ctx->semVision) );
		ok = attachSEM( &(ctx->semVision), ext_vision_sem_id );
	}

	if ( ok > 0 ) {
		initSharedMemory( &(ctx->shmVision), ctx->bufferSize );
		ok = attachSHM( &(ctx->shmVision), ext_vision_shm_id, &(ctx->semVision) );
	}

	if ( ok < 0 ) {
		printf ("Failed to attach on semaphore or shared memory!\n");
		return 0;
	}

	return 1;

}


int ext_vision_comm_readNewDataNb ( ext_vision_comm_s* ctx, SL_Cstate* vData ) {

	if ( readSHMemoryNonBlk( &(ctx->shmVision), ctx->buffer, &(ctx->semVision) ) )
		return 0;


	static int pk_number = -1;
	int buf_num = (int)ctx->buffer[6];
	if ( buf_num == pk_number )
		return 0;

	pk_number = buf_num;

	int i = 1;
	for ( i = _X_; i <= _Z_; ++i ) {
		vData->x[i] = ctx->buffer[i-1];
		vData->xd[i] = ctx->buffer[3+i-1];
	}

	return 1;

}



int ext_vision_comm_readNewDataBlk ( ext_vision_comm_s* ctx, SL_Cstate* vData ) {

	int k = -1;
	while ( k != 0 )
		k = lockData( &(ctx->semSig) );

	
	if ( readSHMemory( &(ctx->shmVision), ctx->buffer, &(ctx->semVision) ) ) 
		return 0;
	

	static int pk_number = -1;
	int buf_num = (int)ctx->buffer[6];
	if ( buf_num == pk_number )
		return 0;

	pk_number = buf_num;

	int i = 1;
	for ( i = _X_; i <= _Z_; ++i ) {
		vData->x[i] = ctx->buffer[i-1];
		vData->xd[i] = ctx->buffer[3+i-1];
	}

	return 1;

}



void ext_vision_calib_init ( ext_vision_calib_s* ctx ) {

	ctx->observ_size = 0; //rows with data
	ctx->observ_max_size = 50000;

	ctx->observ_m = my_matrix(1,ctx->observ_max_size,1,4);
	ctx->robot_m  = my_matrix(1,ctx->observ_max_size,1,3);

	ctx->transf  = my_matrix(1,4,1,4);
	ctx->isTransfValid = 0;

}


void ext_vision_calib_reset ( ext_vision_calib_s* ctx, int realloc ) {

	ctx->observ_size = 0; //rows with data
	if ( realloc ) {
		if ( ctx->observ_m )
			my_free_matrix(ctx->observ_m,1,ctx->observ_max_size,1,4); //3rd and 5th params not used!
		if ( ctx->robot_m )
			my_free_matrix(ctx->robot_m,1,ctx->observ_max_size,1,3);
		ctx->observ_m = my_matrix(1,ctx->observ_max_size,1,4);
		ctx->robot_m  = my_matrix(1,ctx->observ_max_size,1,3);
	}

}


void ext_vision_calib_setMax ( ext_vision_calib_s* ctx, unsigned long newSize, int keepObs ) {

	unsigned long oldSize = ctx->observ_max_size;
	ctx->observ_max_size = newSize;

	if ( keepObs && newSize <= oldSize )
		ctx->observ_size = ctx->observ_size > newSize ? newSize : ctx->observ_size;
	else if ( keepObs ) {
		mat_add_shape( &(ctx->observ_m), newSize-oldSize, 0 );
		mat_add_shape( &(ctx->robot_m), newSize-oldSize, 0 );
	}
	else
		ext_vision_calib_reset( ctx, 1 );

}


int ext_vision_calib_addPoint ( ext_vision_calib_s* ctx, SL_Cstate* vData, SL_Cstate* rData ) {

	if ( ctx->observ_size >= ctx->observ_max_size  )
		return 0;

	int idx = ++ctx->observ_size;

	ctx->observ_m[idx][1] = vData->x[_X_];
	ctx->observ_m[idx][2] = vData->x[_Y_];
	ctx->observ_m[idx][3] = vData->x[_Z_];
	ctx->observ_m[idx][4] = 1.0;

	ctx->robot_m[idx][1] = rData->x[_X_];
	ctx->robot_m[idx][2] = rData->x[_Y_];
	ctx->robot_m[idx][3] = rData->x[_Z_];

	return 1;

}


int ext_vision_calib_calcTrans ( ext_vision_calib_s* ctx ) {

	int ok;

	ctx->observ_m[0][NR] = ctx->observ_size;
	ctx->robot_m[0][NR] = ctx->observ_size;

	Matrix obs_2 = my_matrix(1,4,1,4); // observ_m' * observ_m
	ok = mat_mult_transpose_normal(ctx->observ_m,ctx->observ_m,obs_2);


	ok &= my_inv_ludcmp( obs_2, 4, obs_2 ); // ( observ_m' * observ_m )^-1


	Matrix obs_r = my_matrix(1,4,1,3); // observ_m' * robot_m
	ok &= mat_mult_transpose_normal(ctx->observ_m,ctx->robot_m,obs_r);

	ctx->observ_m[0][NR] = ctx->observ_max_size;
	ctx->robot_m[0][NR] = ctx->observ_max_size;


	Matrix tr_temp = my_matrix(1,4,1,3); // [ rot ; translation ]
	ok &= mat_mult(obs_2,obs_r,tr_temp);


	int i,j;
	for ( i = 1; i <= 3; ++i )
		for ( j = 1; j <= 3; ++j )
			ctx->transf[i][j] = tr_temp[j][i];

	for ( i = 1; i <= 3; ++i ) {
		ctx->transf[i][4] = tr_temp[4][i];
		ctx->transf[4][i] = 0.0;
	}
	ctx->transf[4][4] = 1.0;

	my_free_matrix(obs_2,1,4,1,4);
	my_free_matrix(obs_r,1,4,1,3);
	my_free_matrix(tr_temp,1,4,1,3);


	if ( ! ok )
		printf ("VisionCalib: problem on parameter estimation!\n");
	else
		ctx->isTransfValid = 1;

	return ok;

}


int ext_vision_calib_transPoint ( ext_vision_calib_s* ctx, SL_Cstate* in, SL_Cstate* out ) {

	if ( ! ctx->isTransfValid )
		return 0;

	static Vector v_in = 0;
	static Vector v_out = 0;

	if ( ! v_in) {
		v_in = my_vector(1,4);
		v_out = my_vector(1,4);
	}

	v_in[1] = in->x[_X_];
	v_in[2] = in->x[_Y_];
	v_in[3] = in->x[_Z_];
	v_in[4] = 1.0;

	if ( ! mat_vec_mult(ctx->transf,v_in,v_out) )
		return 0;

	out->x[_X_] = v_out[1];
	out->x[_Y_] = v_out[2];
	out->x[_Z_] = v_out[3];

	return 1;
}


void ext_vision_calib_printTransMatrix ( ext_vision_calib_s* ctx ) {

	if ( ctx->isTransfValid )
		print_mat("VisionCalib: tr matrix",ctx->transf);
	else
		printf("VisionCalib: Tr matrix not set!\n");

}


int ext_vision_calib_exportTransMatrix ( ext_vision_calib_s* ctx, char* filename ) {

	if ( ctx->isTransfValid )
		return fwrite_mat_ascii(filename,ctx->transf);

	return 0;

}


int ext_vision_calib_importTransMatrix ( ext_vision_calib_s* ctx, char* filename ) {

	Matrix tmp = 0;

	int nRows = fread_mat_ascii(filename,&tmp,4);

	if ( nRows != 4 ) {
		if ( nRows != -1 )
			my_free_matrix( tmp, 1, nRows, 1, 1 );
		return 0;
	}

	ctx->transf = tmp;
	ctx->isTransfValid = 1;

	return 1;

}

