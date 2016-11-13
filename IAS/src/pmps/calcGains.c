#include "pmps/calcGains.h"

#include "ias_matrix_utilities.h"


void calcGain_initLocal (int numDim, calcGains_local_s* local_ctx ) {

	local_ctx->numDim = numDim;

	local_ctx->GM = my_matrix ( 1, 2*numDim, 1, 2*numDim );
	local_ctx->M = my_matrix (  1, 2*numDim, 1, 2*numDim );
	local_ctx->AG = my_matrix ( 1, 2*numDim, 1, 2*numDim  );
	local_ctx->Sigma_u_tmp = my_matrix ( 1, 2*numDim , 1, 2*numDim  );
	local_ctx->mask = my_matrix ( 1,  numDim, 1, 2*numDim );
	local_ctx->B_hat = my_matrix ( 1, numDim, 1, numDim );
	local_ctx->B_hat_inv = my_matrix ( 1, numDim, 1, numDim );
	local_ctx->G_inv = my_matrix ( 1,   (int) (local_ctx->GM)[0][NR], 1, (int) (local_ctx->GM)[0][NC] );
	local_ctx->K_temp = my_matrix ( 1,  (int) (local_ctx->M)[0][NR], 1, (int) (local_ctx->GM)[0][NC] );
	local_ctx->K_temp1 = my_matrix ( 1, (int) (local_ctx->mask)[0][NR], 1, (int) (local_ctx->K_temp)[0][NC] );
	local_ctx->ABK = my_matrix ( 1, 2*numDim, 1, 2*numDim );
	local_ctx->ABK_mu = my_matrix ( 1, (int) (local_ctx->ABK)[0][NR], 1, 1 );
	local_ctx->ABK_mu_mask = my_matrix ( 1, (int) local_ctx->mask[0][NR], 1, (int) local_ctx->ABK_mu[0][NC] );

	int i;
	mat_zero ( local_ctx->mask );	  /* [ zeros(numDim), eye(numDim) ]  */
	for ( i = 1; i <= local_ctx->numDim; ++i )
		(local_ctx->mask)[i][i+local_ctx->numDim] = 1.0;

	local_ctx->isInit = TRUE;

}


void calcGain_freeLocal (calcGains_local_s* local_ctx ) {

	if ( local_ctx->isInit == TRUE ) {

		my_basic_free_matrix ( local_ctx->GM );
		my_basic_free_matrix ( local_ctx->M );
		my_basic_free_matrix ( local_ctx->AG );
		my_basic_free_matrix ( local_ctx->Sigma_u_tmp );
		my_basic_free_matrix ( local_ctx->mask );
		my_basic_free_matrix ( local_ctx->B_hat );
		my_basic_free_matrix ( local_ctx->B_hat_inv );
		my_basic_free_matrix ( local_ctx->G_inv );
		my_basic_free_matrix ( local_ctx->K_temp );
		my_basic_free_matrix ( local_ctx->K_temp1 );
		my_basic_free_matrix ( local_ctx->ABK );
		my_basic_free_matrix ( local_ctx->ABK_mu );
		my_basic_free_matrix ( local_ctx->ABK_mu_mask );

		local_ctx->isInit = FALSE;
	}

}


int calcGains( //Matrix w_cov,
//               Matrix Phi_t, Matrix Phi_td,
               Matrix Sigma_t, Matrix Sigma_tD_half,
               Matrix Sigma_x, Matrix Sigma_u,
               Matrix mu_x, Matrix mu_xd,
               Matrix A, Matrix B, Matrix c, double dt,
               Matrix K, Matrix k,
               calcGains_local_s* ctx ) {


	if ( ctx->isInit != TRUE )
		return FALSE;


	int ok = TRUE;


//	ok &= mat_mult ( w_cov, Phi_t, ctx->G_temp );


//	ok &= mat_mult_transpose_normal ( Phi_t, ctx->G_temp, ctx->GM );
//	ok &= mat_add ( ctx->GM, Sigma_x, ctx->GM );                         /* G = Phi_t' * w_cov * Phi_t + Sigma_x; */
	ok &= mat_add ( Sigma_t, Sigma_x, ctx->GM );                         /* G = Sigma_t + Sigma_x; */



//	ok &= mat_mult_transpose_normal ( Phi_td, ctx->G_temp, ctx->M );


	ok &= mat_mult ( A, ctx->GM, ctx->AG );


	ok &= mat_mult_scalar ( Sigma_u, 0.5/dt, ctx->Sigma_u_tmp );


//	ok &= mat_sub ( ctx->M, ctx->AG, ctx->M );
	ok &= mat_sub ( Sigma_tD_half, ctx->AG, ctx->M );
	ok &= mat_sub ( ctx->M, ctx->Sigma_u_tmp, ctx->M );         /* M = Phi_td' * w_cov * Phi_t - A * G - 0.5 * Sigma_u / dt; */


	int i,j;
	for ( i = 1; i <= ctx->numDim; ++i )
		for ( j = 1; j <= ctx->numDim; ++j )
			(ctx->B_hat)[i][j] = B[i+ctx->numDim][j]; /* B_hat = B( numDim+1:end, :); */


	ok &= my_inv_ludcmp ( ctx->B_hat, ctx->numDim, ctx->B_hat_inv);


	ok &= my_inv_ludcmp ( ctx->GM, (int) (ctx->GM)[0][NR], ctx->G_inv);


	ok &= mat_mult ( ctx->M, ctx->G_inv, ctx->K_temp);


	ok &= mat_mult ( ctx->mask, ctx->K_temp, ctx->K_temp1);

	ok &= mat_mult ( ctx->B_hat_inv, ctx->K_temp1, K);
	/* K = B_hat \ [ zeros(numDim), eye(numDim) ]  * M / G; */



	ok &= mat_mult ( B, K, ctx->ABK);
	ok &= mat_add ( A, ctx->ABK, ctx->ABK);


	ok &= mat_mult ( ctx->ABK, mu_x, ctx->ABK_mu);

	ok &= mat_sub ( mu_xd, ctx->ABK_mu, ctx->ABK_mu  );
	ok &= mat_sub ( ctx->ABK_mu, c, ctx->ABK_mu  );

	ok &= mat_mult ( ctx->mask, ctx->ABK_mu, ctx->ABK_mu_mask);

	ok &= mat_mult ( ctx->B_hat_inv, ctx->ABK_mu_mask, k);
	/* k = B_hat \ [ zeros(numDim), eye(numDim) ] * (mu_xd-(A+B*K)*mu_x-c); */


	return ok;

}
