#ifndef BASISGAUSSNORM_H_
#define BASISGAUSSNORM_H_ 1


#include "pmps/basis/basisGeneric.h"

#include "utility.h"

typedef struct {

	Matrix time_mu;
	Matrix at;
	Matrix basis;
	Vector basis_sum_vec;
	Matrix basis_sum;
	Matrix time_mu_sigma;
	Matrix basisD;
	Vector basisD_sum_vec;
	Matrix basisD_sum;
	Matrix basisD_n_a;
	Matrix basisD_n_b;
	Matrix basisD_n_a_min_b;
	Matrix tmp;
	Matrix basisDD;
	Vector basisDD_sum_vec;
	Matrix basisDD_sum;
	Matrix basisDD_n_a;
	Matrix basisDD_n_b1;
	Matrix basisDD_n_b;
	Matrix basisDD_n_c1;
	Matrix tmp1;
	Matrix basisDD_n_c;
	Matrix basisDD_n_d;

} basisGaussNorm_local_s;


typedef struct {

	Matrix mu, sigma;

	basisGaussNorm_local_s local_ctx;
	int isLocalCtx_init;

} basisGaussNorm_ctx_s;


void basisGaussNorm_freeLocal ( basisGaussNorm_ctx_s* ctx );

/* time, mu, sigma just for allocation the proper size, their values are not read.*/
int basisGaussNorm_init ( Matrix time, Matrix mu, Matrix sigma, struct basis_gen_ctx_s* ctx );

void basisGaussNorm_free ( struct basis_gen_ctx_s* ctx );

int basisGaussNorm ( Matrix time, struct basis_gen_ctx_s* ctx );


#endif /* BASISGAUSSNORM_H_ */
