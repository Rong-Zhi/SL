#ifndef BASISVONMISES_H_
#define BASISVONMISES_H_ 1

#include "pmps/basis/basisGeneric.h"

#include "utility.h"

typedef struct {

	Matrix phase_mu_2pf;
	Matrix basis;
	Matrix basisDtmp;
	Matrix basisD;
	Matrix basisDDtmp;
	Matrix basisDD;
	Vector basis_sum_vec;
	Matrix basis_sum;
	Matrix basisD_sum;
	Matrix basisDD_sum;
	Matrix basisD_n_tmp;
	Matrix basisD_n_tmp1;
	Matrix basis_sum_sqr_inv;
	Matrix basisD_sum_sqr;
	Matrix basis_sum_cub_inv;
	Matrix basisDD_n_tmp2;

} vonMisesLocalCtx_s;


typedef struct {

	Matrix mu;
	double k, f;

	vonMisesLocalCtx_s local_ctx;
	int isLocalCtx_init;

} vonMisesCtx_s;


int basisVonMises_init ( Matrix phase, Matrix mu, double k, double f, struct basis_gen_ctx_s* ctx);

void basisVonMises_free ( struct basis_gen_ctx_s* ctx_);

int basisVonMises ( Matrix phase, struct basis_gen_ctx_s* ctx );






#endif /* BASISVONMISES_H_ */
