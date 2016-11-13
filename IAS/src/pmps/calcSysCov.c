#include "pmps/calcSysCov.h"


#include "pmps/calcPhi.h"
#include "ias_matrix_utilities.h"




int calcSysCovBasic_init ( int numDim, calcSysCovCtx_s* ctx ) {

	int ok = TRUE;

	ctx->numDim = numDim;
	ctx->numBasis = 0;

	ctx->mu_x   = my_matrix ( 1, 2 * numDim, 1, 1 );
	ctx->mu_xd  = my_matrix ( 1, 2 * numDim, 1, 1 );


	ctx->Sigma_t       = my_matrix ( 1, numDim*2,  1, numDim*2 );
	ctx->Sigma_t1      = my_matrix ( 1, numDim*2,  1, numDim*2 );
	ctx->Sigma_t_t1    = my_matrix ( 1, numDim*2,  1, numDim*2 );
	ctx->Sigma_td_half = my_matrix ( 1, numDim*2,  1, numDim*2 );



	return ok;
}


void calcSysCovBasic_free ( calcSysCovCtx_s* ctx ) {

	my_basic_free_matrix ( ctx->mu_x );
	my_basic_free_matrix ( ctx->mu_xd );


	my_basic_free_matrix ( ctx->Sigma_t );
	my_basic_free_matrix ( ctx->Sigma_t1 );
	my_basic_free_matrix ( ctx->Sigma_t_t1 );
	my_basic_free_matrix ( ctx->Sigma_td_half );

}



int calcSysCovBasic_addBasisGauss ( calcSysCovCtx_s* ctx, Matrix time, Matrix mu_w, Matrix cov_w, Matrix mu, Matrix sigma ) {

	int ok = TRUE;

	int idx = ctx->numBasis +1 ;

	ctx->mu_w[idx] = my_matrix( 1, (int) mu_w[0][NR], 1, (int) mu_w[0][NC] );
    ctx->cov_w[idx] = my_matrix( 1, (int) cov_w[0][NR], 1, (int) cov_w[0][NC] );

    ok &= mat_equal ( mu_w, ctx->mu_w[idx] );
    ok &= mat_equal ( cov_w,ctx->cov_w[idx] );


	ok &= basisGaussNorm_init(time,mu,sigma, &(ctx->basis[idx]) );

	if ( ok == TRUE )
		ctx->numBasis += 1;


	return ok;
}



int calcSysCovBasic_addBasisVonMises ( calcSysCovCtx_s* ctx, Matrix time, Matrix mu_w, Matrix cov_w, Matrix mu, double k, double f ) {
	int ok = TRUE;

	int idx = ctx->numBasis +1 ;

	ctx->mu_w[idx] = mu_w;  //TODO check this?
	ctx->cov_w[idx] = cov_w;

	ok &= basisVonMises_init ( time, mu, k, f, &(ctx->basis[idx]) );

	if ( ok == TRUE )
		ctx->numBasis += 1;

	return ok;
}








int calcSysCovBasic ( Matrix timeM, calcSysCovCtx_s* ctx ) {

	int ok = TRUE;

	static Matrix Phi_t, Phi_t1, Phi_td;
	static Matrix Sigma_t_tmp;


	static int firsttime = TRUE;

	if ( firsttime == TRUE ) {
		firsttime = FALSE;


		int numBasis = (int) ctx->mu_w[1][0][NR];  // 2*numBasis!

		Phi_t  = my_matrix ( 1, numBasis, 1, 2 * ctx->numDim );
		Phi_t1 = my_matrix ( 1, numBasis , 1, 2 * ctx->numDim );
		Phi_td = my_matrix ( 1, numBasis , 1, 2 * ctx->numDim );

		Sigma_t_tmp = my_matrix( 1, numBasis, 1, 2 * ctx->numDim  );

	}


	ok &= ctx->basis[1].bEval( timeM, &(ctx->basis[1]) );

	ok &= calcPhi( ctx->basis[1].basis_n, ctx->basis[1].basisD_n, ctx->basis[1].basisDD_n, 1, ctx->numDim, /* function inputs */
			       Phi_t, Phi_t1, Phi_td ); /* function outputs */


	ok &= mat_mult_transpose_normal( Phi_t,  ctx->mu_w[1], ctx->mu_x ); /* mu_x = Phi_t' * w_mu; */
	ok &= mat_mult_transpose_normal( Phi_td, ctx->mu_w[1], ctx->mu_xd ); /* mu_xd = Phi_td' * w_mu; */



	ok &= mat_mult( ctx->cov_w[1], Phi_t,   Sigma_t_tmp );
	ok &= mat_mult_transpose_normal( Phi_t, Sigma_t_tmp, ctx->Sigma_t );
//	ok &= mat_add( ctx->Sigma_t, Sigma_x_est, ctx->Sigma_t ); /* Sigma_t = Phi_t' * w_cov * Phi_t + Sigma_x; */


//	ok &= mat_mult( ctx->cov_w[1], Phi_t,  Sigma_t_tmp );
	ok &= mat_mult_transpose_normal( Phi_td, Sigma_t_tmp, ctx->Sigma_td_half); /* Sigma_td_half =  Phi_td' * w_cov * Phi_t; */



	ok &= mat_mult( ctx->cov_w[1], Phi_t1,  Sigma_t_tmp );
	ok &= mat_mult_transpose_normal( Phi_t1, Sigma_t_tmp, ctx->Sigma_t1 );


//	ok &= mat_add( ctx->Sigma_t1, Sigma_x_est, ctx->Sigma_t1 ); /* Sigma_t1 = Phi_t1' * w_cov * Phi_t1 + Sigma_x; */

//	ok &= mat_mult( ctx->cov_w[1], Phi_t1,  Sigma_t_tmp );
	ok &= mat_mult_transpose_normal( Phi_t, Sigma_t_tmp, ctx->Sigma_t_t1 ); /* Sigma_t_t1 = Phi_t' * w_cov * Phi_t1; */



	return ok;

}


int calcSysCovPreAlloc ( Matrix timeM, calcSysCovCtx_s* ctx ) {

	int idx = timeM[1][1];

	if ( idx > ((int) ctx->store_Sigma_t[0][NR] -1  ) ) //TODO ????
		return FALSE;


	int i,j;

	for ( i = 1; i <= 2*ctx->numDim; ++i ) {

		ctx->mu_x [i][1] =  ctx->store_mu_x[idx][i];
		ctx->mu_xd [i][1] = ctx->store_mu_xd[idx][i];

		for ( j = 1; j <= 2*ctx->numDim; ++j ) {


			ctx->Sigma_t [i][j] = ctx->store_Sigma_t[idx][(i-1)*2*ctx->numDim+j];
			ctx->Sigma_t1[i][j] = ctx->store_Sigma_t[idx+1][(i-1)*2*ctx->numDim+j];

			ctx->Sigma_t_t1[i][j] = ctx->store_Sigma_t_t1[idx][(i-1)*2*ctx->numDim+j];

			ctx->Sigma_td_half[i][j] = ctx->store_Sigma_td_half[idx][(i-1)*2*ctx->numDim+j];

		}
	}


	return TRUE;
}
