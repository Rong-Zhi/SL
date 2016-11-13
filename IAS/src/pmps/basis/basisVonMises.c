#include "pmps/basis/basisVonMises.h"

#include "pmps/bsxfun.h"
#include "ias_matrix_utilities.h"

#include <math.h>


static double my_inv ( double a) {
	return 1.0 / a;
}


static double f1 ( double a) {
	return 1.0 / (a*a);
}


static double f2 ( double a) {
	return 2.0*a;
}


static double f3 ( double a) {
	return (a*a);
}

static double f4 ( double a) {
	return 1.0 / (a*a*a);
}




int basisVonMises_init ( Matrix phase, Matrix mu, double k, double f, struct basis_gen_ctx_s* gen_ctx)  {

	int ok = TRUE;

	gen_ctx->bEval = basisVonMises;

	gen_ctx->basis_ctx = malloc( sizeof(vonMisesCtx_s) );

	if ( gen_ctx->basis_ctx == 0 )
		return FALSE;



	vonMisesCtx_s* ctx_ = gen_ctx->basis_ctx;


	ctx_->f = f;
	ctx_->k = k;


	vonMisesLocalCtx_s* ctx = &(ctx_->local_ctx);




	ok &= bsxfun_malloc ( phase, mu, &(ctx->phase_mu_2pf) );

	ctx->basis      = my_matrix( 1, (int) (ctx->phase_mu_2pf)[0][NR], 1, (int) (ctx->phase_mu_2pf)[0][NC]);
	ctx->basisDtmp  = my_matrix( 1, (int) (ctx->phase_mu_2pf)[0][NR], 1, (int) (ctx->phase_mu_2pf)[0][NC]);
	ctx->basisD     = my_matrix( 1, (int) (ctx->phase_mu_2pf)[0][NR], 1, (int) (ctx->phase_mu_2pf)[0][NC]);
	ctx->basisDDtmp = my_matrix( 1, (int) (ctx->phase_mu_2pf)[0][NR], 1, (int) (ctx->phase_mu_2pf)[0][NC]);
	ctx->basisDD    = my_matrix( 1, (int) (ctx->phase_mu_2pf)[0][NR], 1, (int) (ctx->phase_mu_2pf)[0][NC]);


	ctx->basis_sum_vec = my_vector ( 1, (int) (ctx->basis)[0][NR]);
	ctx->basis_sum     = my_matrix(1, (int) (ctx->basis)[0][NR], 1, 1 );
	ctx->basisD_sum    = my_matrix(1, (int) (ctx->basis)[0][NR], 1, 1 );
	ctx->basisDD_sum   = my_matrix(1, (int) (ctx->basis)[0][NR], 1, 1 );


	ok &= bsxfun_malloc ( ctx->basisD,ctx-> basis_sum, &(ctx->basisD_n_tmp) );
	ok &= bsxfun_malloc ( ctx->basis, ctx->basisD_sum, &(ctx->basisD_n_tmp1) );


	ctx->basis_sum_sqr_inv = my_matrix ( 1, (int) (ctx->basis_sum)[0][NR], 1, (int) (ctx->basis_sum)[0][NC] );
	ctx->basisD_sum_sqr    = my_matrix ( 1, (int) (ctx->basisD_sum)[0][NR], 1, (int) (ctx->basisD_sum)[0][NC] );
	ctx->basis_sum_cub_inv = my_matrix ( 1, (int) (ctx->basis_sum)[0][NR], 1, (int) (ctx->basis_sum)[0][NC] );


	ok &= bsxfun_malloc ( ctx->basisD, ctx->basisD_sum, &(ctx->basisDD_n_tmp2) );


	ctx_->mu = my_matrix( 1, (int) mu[0][NR], 1, (int) mu[0][NC] );

	int i,j;

	for ( i = 1; i <= (int) mu[0][NR]; ++i )
		for ( j = 1; j <= (int) mu[0][NC]; ++j )
			(ctx_->mu)[i][j] = mu[i][j];



	ok &= bsxfun_malloc ( ctx->basis, ctx->basis_sum, &(gen_ctx->basis_n) );
	ok &= bsxfun_malloc ( ctx->basisD_n_tmp, ctx->basis_sum, &(gen_ctx->basisD_n) );
	ok &= bsxfun_malloc ( ctx->basisDD, ctx->basis_sum, &(gen_ctx->basisDD_n) );



	ctx_->isLocalCtx_init = ok;



	return ok;


}



void basisVonMises_free ( struct basis_gen_ctx_s* gen_ctx) {

	vonMisesCtx_s* ctx_ = gen_ctx->basis_ctx;

	if ( ctx_->isLocalCtx_init != TRUE )
		return;

	ctx_->isLocalCtx_init = FALSE;


	my_basic_free_matrix( ctx_->mu );

	vonMisesLocalCtx_s* ctx = &(ctx_->local_ctx);

	my_basic_free_matrix ( ctx->phase_mu_2pf );

	my_basic_free_matrix ( ctx->basis );

	my_basic_free_matrix ( ctx->basisDtmp );
	my_basic_free_matrix ( ctx->basisD  );
	my_basic_free_matrix ( ctx->basisDDtmp );
	my_basic_free_matrix ( ctx->basisDD );

	my_free_vector ( ctx->basis_sum_vec, 1, ctx->basis_sum_vec[NR] );

	my_basic_free_matrix ( ctx->basis_sum );
	my_basic_free_matrix ( ctx->basisD_sum  );
	my_basic_free_matrix ( ctx->basisDD_sum );


	my_basic_free_matrix ( ctx->basisD_n_tmp );
	my_basic_free_matrix ( ctx->basisD_n_tmp1 );


	ctx->basis_sum_sqr_inv = my_matrix ( 1, (int) (ctx->basis_sum)[0][NR], 1, (int) (ctx->basis_sum)[0][NC] );
	ctx->basisD_sum_sqr    = my_matrix ( 1, (int) (ctx->basisD_sum)[0][NR], 1, (int) (ctx->basisD_sum)[0][NC] );
	ctx->basis_sum_cub_inv = my_matrix ( 1, (int) (ctx->basis_sum)[0][NR], 1, (int) (ctx->basis_sum)[0][NC] );

	my_basic_free_matrix ( ctx->basisDD_n_tmp2 );



	my_basic_free_matrix ( gen_ctx->basis_n );
	my_basic_free_matrix ( gen_ctx->basisD_n );
	my_basic_free_matrix ( gen_ctx->basisDD_n );

	free(gen_ctx->basis_ctx);

	gen_ctx->basis_ctx = 0;

}






int basisVonMises ( Matrix phase, struct basis_gen_ctx_s* gen_ctx ) {


	Matrix basis_n   = gen_ctx->basis_n;
	Matrix basisD_n  = gen_ctx->basisD_n;
	Matrix basisDD_n = gen_ctx->basisDD_n;

	vonMisesCtx_s* ctx = gen_ctx->basis_ctx;

	vonMisesLocalCtx_s* lctx = &(ctx->local_ctx);

	int ok = TRUE;

	ok &= bsxfun ( my_Minus, phase, ctx->mu, lctx->phase_mu_2pf, 0, 0 );   /*  phase_mu = bsxfun(@minus, phase, mu );  */
	ok &= mat_mult_scalar( lctx->phase_mu_2pf, 2*PI* (ctx->f), lctx->phase_mu_2pf);



	ok &= mat_equal_apply_math( lctx->phase_mu_2pf, cos, lctx->basis);
	ok &= mat_mult_scalar( lctx->basis, ctx->k, lctx->basis);
	ok &= mat_add_scalar(  lctx->basis, -(ctx->k), lctx->basis);
	ok &= mat_equal_apply_math( lctx->basis, exp, lctx->basis);   /* basis = exp ( k * cos ( phase_mu * 2 * pi * f ) - k ); */



	ok &= mat_equal_apply_math( lctx->phase_mu_2pf, sin, lctx->basisDtmp);
	ok &= mat_mult_scalar( lctx->basisDtmp, -2*PI*  (ctx->k)  * (ctx->f), lctx->basisDtmp);
	ok &= mat_dmult (lctx->basis, lctx->basisDtmp, lctx->basisD);           /*   basisD = - basis * k .* sin ( phase_mu * 2 * pi * f) * 2 * pi; */



	ok &= mat_dmult (lctx->basisD, lctx->basisDtmp, lctx->basisDD);

	ok &= mat_equal_apply_math( lctx->phase_mu_2pf, cos, lctx->basisDDtmp);
	ok &= mat_mult_scalar(lctx->basisDDtmp, -4*PI*PI* (ctx->k) * (ctx->f) * (ctx->f), lctx->basisDDtmp);
	ok &= mat_dmult (lctx->basis, lctx->basisDDtmp, lctx->basisDDtmp);

	ok &= mat_add(lctx->basisDD,lctx->basisDDtmp,lctx->basisDD);           /* basisDD = - basisD * k .* sin ( phase_mu * 2 * pi * f ) * 2 * pi ...
	                                                        - basis  * k .* cos ( phase_mu * 2 * pi * f ) * 4 * pi^2;        */


	ok &= mat_sum_columns( lctx->basis, lctx->basis_sum_vec );

	vec_to_mat_op ( lctx->basis_sum_vec, lctx->basis_sum, 0 );   /* sum(basis,2); */



	ok &= mat_sum_columns( lctx->basisD, lctx->basis_sum_vec );
	vec_to_mat_op ( lctx->basis_sum_vec, lctx->basisD_sum, 0 );   /* sum(basisD,2); */



	ok &= mat_sum_columns( lctx->basisDD, lctx->basis_sum_vec );
	vec_to_mat_op ( lctx->basis_sum_vec, lctx->basisDD_sum, 0 );   /* sum(basisDD,2); */



	ok &= bsxfun ( my_Times, lctx->basis, lctx->basis_sum, basis_n, 0, my_inv );  /*  basis_n = bsxfun ( @rdivide, basis , sum(basis,2) ); */

	ok &= bsxfun ( my_Times, lctx->basisD, lctx->basis_sum, lctx->basisD_n_tmp, 0, 0 );   /* basisD_n = bsxfun ( @times, basisD , sum(basis,2) ) ; */


	ok &= bsxfun ( my_Times, lctx->basis, lctx->basisD_sum, lctx->basisD_n_tmp1, 0, 0 );   /* bsxfun ( @times, basis , sum(basisD,2) ) */


	ok &= mat_sub ( lctx->basisD_n_tmp, lctx->basisD_n_tmp1, lctx->basisD_n_tmp );  /* basisD_n = basisD_n - bsxfun ( @times, basis , sum(basisD,2) ) ; */


	ok &= bsxfun ( my_Times, lctx->basisD_n_tmp, lctx->basis_sum, basisD_n, 0, f1 );  /* basisD_n = bsxfun ( @rdivide, basisD_n , sum(basis,2).^2 ) ; */




	ok &= mat_equal_apply_math(lctx->basis_sum, f1, lctx->basis_sum_sqr_inv );
	ok &= mat_equal_apply_math(lctx->basisD_sum, f3, lctx->basisD_sum_sqr );
	ok &= mat_equal_apply_math(lctx->basis_sum, f4, lctx->basis_sum_cub_inv );



	ok &= bsxfun ( my_Times, lctx->basisDD, lctx->basis_sum, basisDD_n, 0, my_inv );  /* basisDD_n = bsxfun ( @rdivide, basisDD , sum(basis,2) ); */


	ok &= mat_dmult ( lctx->basisD_sum, lctx->basis_sum_sqr_inv, lctx->basisD_sum);


	ok &= bsxfun ( my_Times, lctx->basisD, lctx->basisD_sum, lctx->basisDD_n_tmp2, f2, 0 );

	ok &= mat_sub( basisDD_n, lctx->basisDD_n_tmp2, basisDD_n );  /* basisDD_n = basisDD_n -  bsxfun ( @times, 2 .* basisD , sum(basisD,2) ./ (sum(basis,2)).^2  ); */


	ok &= mat_dmult ( lctx->basisDD_sum, lctx->basis_sum_sqr_inv, lctx->basisDD_sum);
	ok &= bsxfun ( my_Times, lctx->basis, lctx->basisDD_sum, lctx->basisDD_n_tmp2, 0, 0 );

	ok &= mat_sub( basisDD_n, lctx->basisDD_n_tmp2, basisDD_n );  /* basisDD_n = basisDD_n -  bsxfun ( @times, basis , sum(basisDD,2) ./ (sum(basis,2)).^2  ); */


	ok &= mat_dmult ( lctx->basisD_sum_sqr, lctx->basis_sum_cub_inv, lctx->basisD_sum_sqr);
	ok &= bsxfun ( my_Times, lctx->basis, lctx->basisD_sum_sqr, lctx->basisDD_n_tmp2, f2, 0 );

	ok &= mat_add( basisDD_n, lctx->basisDD_n_tmp2, basisDD_n );  /* basisDD_n = basisDD_n +  bsxfun ( @times, 2.* basis , sum(basisD,2).^2 ./ (sum(basis,2)).^3  ); */

	return ok;

}
