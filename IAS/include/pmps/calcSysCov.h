#ifndef CALCSYSCOV_H_
#define CALCSYSCOV_H_ 1


#include "pmps/basis/basisGeneric.h"
#include "pmps/basis/basisGaussNorm.h"
#include "pmps/basis/basisVonMises.h"

#include "utility.h"



typedef struct {

	int numDim;

	// Storage for generating Sigmas
	int numBasis;
	Matrix mu_w[50];
	Matrix cov_w[50];
	struct basis_gen_ctx_s basis[50];  //how to init the shit? TODO


	Matrix store_mu_x, store_mu_xd;    //how to init the shit? TODO
	Matrix store_Sigma_t, store_Sigma_t_t1;
	Matrix store_Sigma_td_half;

    // Output storage for the functions
	Matrix mu_x, mu_xd;
	Matrix Sigma_t, Sigma_t1, Sigma_t_t1;
	Matrix Sigma_td_half;


} calcSysCovCtx_s;


int  calcSysCovBasic_init ( int numDim, calcSysCovCtx_s* ctx );

void calcSysCovBasic_free ( calcSysCovCtx_s* ctx );


int calcSysCovBasic_addBasisGauss ( calcSysCovCtx_s* ctx, Matrix time, Matrix mu_w,
                                    Matrix cov_w, Matrix mu, Matrix sigma );

int calcSysCovBasic_addBasisVonMises ( calcSysCovCtx_s* ctx, Matrix time, Matrix mu_w,
                                       Matrix cov_w, Matrix mu, double k, double f );


int  calcSysCovBasic ( Matrix timeM, calcSysCovCtx_s* ctx );

int  calcSysCovPreAlloc ( Matrix timeM, calcSysCovCtx_s* ctx );




typedef  int  (*calcSysCovFunc) ( Matrix , calcSysCovCtx_s* );




#endif /* CALCSYSCOV_H_ */
