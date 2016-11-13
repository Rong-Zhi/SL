#include "pmps/pmps.h"

#include "utility.h"

#include "pmps/calcPhi.h"
#include "ias_matrix_utilities.h"
#include "pmps/system/linearSystem.h"





int pmps_init ( int numDim, double dt, pmps_ctx* ctx ) {

	if ( ctx->isInit == TRUE )
		return FALSE;

	int ok = TRUE;

	ctx->isInit = TRUE;

	ctx->numDimensions = numDim;

	ctx->dt = dt;


	ok &= calcSysCovBasic_init ( numDim, &(ctx->calcSysCov_ctx) );


//	ctx->mu_x   = my_matrix ( 1, 2 * numDim, 1, 1 );
//	ctx->mu_xd  = my_matrix ( 1, 2 * numDim, 1, 1 );
//
//
//	ctx->Sigma_t       = my_matrix ( 1, numDim*2,  1, numDim*2 );
//	ctx->Sigma_t1      = my_matrix ( 1, numDim*2,  1, numDim*2 );
//	ctx->Sigma_t_t1    = my_matrix ( 1, numDim*2,  1, numDim*2 );
//	ctx->Sigma_td_half = my_matrix ( 1, numDim*2,  1, numDim*2 );


	ctx->Sigma_x = my_matrix ( 1, numDim*2,  1, numDim*2 );
	mat_zero ( ctx->Sigma_x );   /*  Sigma_x = zeros(numDim*2); */

	ctx->Sigma_u = my_matrix ( 1, numDim*2,  1, numDim*2 );


	ok &= calcSysNoise_initLocal ( numDim, &(ctx->sysNoise_ctx));

	ctx->A = my_matrix(1, 2*numDim, 1, 2*numDim );
	ctx->B = my_matrix(1, 2*numDim, 1, numDim );
	ctx->c = my_matrix(1, 2*numDim, 1, 1 );

	calcGain_initLocal ( numDim, &(ctx->calcGains_ctx) );

	return TRUE;

}



int pmps_free (  pmps_ctx* ctx ) {

	if ( ctx->isInit != TRUE )
		return FALSE;

	ctx->isInit = FALSE;

//	my_basic_free_matrix( ctx->mu_x  );
//	my_basic_free_matrix( ctx->mu_xd );
//
//
//	my_basic_free_matrix( ctx->Sigma_t );
//	my_basic_free_matrix( ctx->Sigma_t1 );
//	my_basic_free_matrix( ctx->Sigma_t_t1 );
//	my_basic_free_matrix( ctx->Sigma_td_half );


	my_basic_free_matrix( ctx->Sigma_x );
	my_basic_free_matrix( ctx->Sigma_u );

	calcSysNoise_freeLocal( &(ctx->sysNoise_ctx) );

	my_basic_free_matrix( ctx->A );
	my_basic_free_matrix( ctx->B );
	my_basic_free_matrix( ctx->c );

	calcGain_freeLocal( &(ctx->calcGains_ctx) );

	return TRUE;

}



int pmps_execute_step (pmps_ctx* ctx, Matrix time,
                       Matrix K_out, Matrix k_out )
{

	int ok = TRUE;


	ok &= ctx->calcSysCovF ( time, &(ctx->calcSysCov_ctx ));



	ok &= calcSysNoise ( ctx->calcSysCov_ctx.Sigma_t, ctx->calcSysCov_ctx.Sigma_t1, ctx->calcSysCov_ctx.Sigma_t_t1,
						 ctx->Sigma_u, &(ctx->sysNoise_ctx) );

	/* store_sigma_u(i,:) = diag(Sigma_u(numDim+1:end,numDim+1:end)); */



	ok &= linearSystemMat ( 0, ctx->A, ctx->B, ctx->c );    /* [ A B c ] = properties.sysFunc ( q_test ); */



	ok &= calcGains( ctx->calcSysCov_ctx.Sigma_t, ctx->calcSysCov_ctx.Sigma_td_half,
					 ctx->Sigma_x, ctx->Sigma_u,
					 ctx->calcSysCov_ctx.mu_x, ctx->calcSysCov_ctx.mu_xd, ctx->A, ctx->B, ctx->c, ctx->dt,
					 K_out, k_out, &(ctx->calcGains_ctx) );


//	print_mat("Sigma_t", ctx->calcSysCov_ctx.Sigma_t );
//	print_mat("Sigma_td_half", ctx->calcSysCov_ctx.Sigma_td_half );
//	print_mat("Sigma_x", ctx->Sigma_x );
//	print_mat("Sigma_u",ctx->Sigma_u );
//	print_mat("mu_x", ctx->calcSysCov_ctx.mu_x );
//	print_mat("mu_xd", ctx->calcSysCov_ctx.mu_xd );
//
//	print_mat("K_out", K_out );
//	print_mat("k_out", k_out );

//	return FALSE;


	return ok;

}



//int pmps_execute_step (int i /* the step idx */ ,   /* remove i, pass pointers for i and i+1 steps */
//                       pmps_ctx* cxt,
//                       Matrix basis, Matrix basisD, Matrix basisDD,
//                       Matrix w_mu, Matrix w_cov,
//                       Matrix K_out, Matrix k_out )
//{
//
//	int ok = TRUE;
//
//	ok &= calcPhi ( basis, basisD, basisDD, i, cxt->numDimensions,  /* function inputs */
//			        cxt->Phi_t, cxt->Phi_t1, cxt->Phi_td );                        /* function outputs */
//
//
//
//	ok &= mat_mult_transpose_normal ( cxt->Phi_t,  w_mu, cxt->mu_x  );     /* mu_x = Phi_t' * w_mu; */
//	ok &= mat_mult_transpose_normal ( cxt->Phi_td, w_mu, cxt->mu_xd );     /* mu_xd = Phi_td' * w_mu; */
//
//
//	ok &= calcSysNoise ( cxt->Phi_t, cxt->Phi_t1, w_cov,
//						 cxt->Sigma_x, cxt->Sigma_u, &(cxt->sysNoise_ctx) );
//
//	/* store_sigma_u(i,:) = diag(Sigma_u(numDim+1:end,numDim+1:end)); */
//
//
//
//	ok &= linearSystemMat ( 0, cxt->A, cxt->B, cxt->c );    /* [ A B c ] = properties.sysFunc ( q_test ); */
//
//
//	//w_cov, cxt->Phi_t, cxt->Phi_td,
//
//	Sigma_t;
//	Sigma_td_half;
//
//	ok &= calcGains( cxt->Sigma_x, cxt->Sigma_u,
//					 cxt->mu_x, cxt->mu_xd, cxt->A, cxt->B, cxt->c, cxt->dt,
//					 K_out, k_out, &(cxt->calcGains_ctx) );
//
//
//	return ok;
//
//}
