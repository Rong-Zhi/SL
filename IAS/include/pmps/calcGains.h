#ifndef CALCGAIN_H_
#define CALCGAIN_H_ 1


#include "utility.h"

typedef struct {

	Matrix GM;
	Matrix M;
	Matrix AG;
	Matrix Sigma_u_tmp;
	Matrix mask;
	Matrix B_hat;
	Matrix B_hat_inv;
	Matrix G_inv;
	Matrix K_temp;
	Matrix K_temp1;
	Matrix ABK;
	Matrix ABK_mu;
	Matrix ABK_mu_mask;

	int numDim;

	int isInit;

} calcGains_local_s;


void calcGain_initLocal (int numDim, calcGains_local_s* local_ctx );

void calcGain_freeLocal (calcGains_local_s* local_ctx );

int calcGains( Matrix Sigma_t, Matrix Sigma_tD_half,
               Matrix Sigma_x, Matrix Sigma_u,
               Matrix mu_x, Matrix mu_xd,
               Matrix A, Matrix B, Matrix c, double dt,
               Matrix K, Matrix k,
               calcGains_local_s* ctx ) ;



#endif /* CALCGAIN_H_ */
