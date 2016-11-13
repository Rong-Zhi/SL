#ifndef CALCSYSNOISE_H_
#define CALCSYSNOISE_H_ 1

#include "utility.h"

typedef struct {

	Matrix Sigma_t_inv;
	Matrix Sigma_x_tmp;

	int isInit;

} calcSysNoise_local_s;

int calcSysNoise_initLocal ( int ndim, calcSysNoise_local_s* ctx);

void calcSysNoise_freeLocal ( calcSysNoise_local_s* ctx);


int calcSysNoise( Matrix Sigma_t, Matrix  Sigma_t1, Matrix  Sigma_t_t1,
                  Matrix Sigma_x_out, calcSysNoise_local_s* ctx );


#endif /* CALCSYSNOISE_H_ */
