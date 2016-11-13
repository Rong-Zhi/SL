#ifndef PMPS_H_
#define PMPS_H_ 1

#include "utility.h"

#include "pmps/calcSysCov.h"
#include "pmps/calcSysNoise.h"
#include "pmps/calcGains.h"

typedef struct {

	int isInit;

	int numDimensions;
	double dt;


	//Local stuff

//	Matrix mu_x, mu_xd;
//	Matrix Sigma_t, Sigma_t1, Sigma_t_t1, Sigma_td_half;

	Matrix Sigma_x, Sigma_u;

	Matrix A, B, c;


	calcSysCovFunc calcSysCovF;  //TODO how to init?


	calcSysCovCtx_s  calcSysCov_ctx;
	calcSysNoise_local_s sysNoise_ctx;
	calcGains_local_s calcGains_ctx;



} pmps_ctx;




int pmps_init ( int numDim, double dt, pmps_ctx* ctx );

int pmps_free (pmps_ctx* ctx);

int pmps_execute_step (pmps_ctx* ctx, Matrix time,
                       Matrix K_out, Matrix k_out );

//int pmps_execute_step (int i /* the step idx */ ,
//                       pmps_ctx* cxt,
//                       Matrix basis, Matrix basisD, Matrix basisDD,
//                       Matrix w_mu, Matrix w_cov,
//                       Matrix K_out, Matrix k_out );



#endif /* PMPS_H_ */
