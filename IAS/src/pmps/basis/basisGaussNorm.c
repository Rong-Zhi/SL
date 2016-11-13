#include "pmps/basis/basisGaussNorm.h"


#include "pmps/bsxfun.h"
#include "ias_matrix_utilities.h"

#include <math.h>




static int basisGaussNorm_initLocal ( Matrix time, Matrix mu, Matrix sigma,
                               basisGaussNorm_local_s* local_ctx )
{

	int ok = TRUE;

	ok &= bsxfun_malloc ( time, mu, &(local_ctx->time_mu) );
	ok &= bsxfun_malloc ( local_ctx->time_mu, sigma, &(local_ctx->at) );
	ok &= bsxfun_malloc ( local_ctx->at, sigma, &(local_ctx->basis) );
	local_ctx->basis_sum_vec = my_vector ( 1, (int) (local_ctx->basis)[0][NR]);
	local_ctx->basis_sum = my_matrix ( 1, (int) (local_ctx->basis)[0][NR], 1, 1 );
	ok &= bsxfun_malloc ( local_ctx->time_mu, sigma, &(local_ctx->time_mu_sigma) );
	local_ctx->basisD = my_matrix ( 1, (int) (local_ctx->time_mu_sigma)[0][NR], 1, (int) (local_ctx->time_mu_sigma)[0][NC] );
	local_ctx->basisD_sum_vec = my_vector ( 1, (int) (local_ctx->basisD)[0][NR]);
	local_ctx->basisD_sum = my_matrix ( 1, (int) (local_ctx->basisD)[0][NR], 1, 1 );
	ok &= bsxfun_malloc ( local_ctx->basisD, local_ctx->basis_sum, &(local_ctx->basisD_n_a) );
	ok &= bsxfun_malloc ( local_ctx->basis, local_ctx->basisD_sum, &(local_ctx->basisD_n_b) );
	local_ctx->basisD_n_a_min_b = my_matrix ( 1, (int) (local_ctx->basisD_n_b)[0][NR], 1, (int) (local_ctx->basisD_n_b)[0][NC] );
	ok &= bsxfun_malloc ( local_ctx->basis, sigma, &(local_ctx->tmp) );
	local_ctx->basisDD = my_matrix ( 1, (int) (local_ctx->basisD)[0][NR], 1, (int) (local_ctx->basisD)[0][NC] );
	local_ctx->basisDD_sum_vec = my_vector ( 1, (int) (local_ctx->basisDD)[0][NR]);
	local_ctx->basisDD_sum = my_matrix ( 1, (int) (local_ctx->basisDD)[0][NR], 1, 1 );
	ok &= bsxfun_malloc ( local_ctx->basisDD, local_ctx->basis_sum, &(local_ctx->basisDD_n_a) );
	ok &= bsxfun_malloc ( local_ctx->basisD, local_ctx->basis_sum, &(local_ctx->basisDD_n_b1) );
	ok &= bsxfun_malloc ( local_ctx->basisDD_n_b1, local_ctx->basisD_sum, &(local_ctx->basisDD_n_b) );
	local_ctx->basisDD_n_c1 = my_matrix ( 1, (int) (local_ctx->basis_sum)[0][NR], 1, (int) (local_ctx->basis_sum)[0][NC] );
	local_ctx->tmp1 = my_matrix ( 1, (int) (local_ctx->basisD_sum)[0][NR], 1, (int) (local_ctx->basisD_sum)[0][NC] );
	ok &= bsxfun_malloc ( local_ctx->basis, local_ctx->basisDD_n_c1, &(local_ctx->basisDD_n_c) );
	local_ctx->basisDD_n_d = my_matrix ( 1, (int) (local_ctx->basisDD_n_c)[0][NR], 1, (int) (local_ctx->basisDD_n_c)[0][NC] );

	return ok;

}


void basisGaussNorm_freeLocal ( basisGaussNorm_ctx_s* ctx ) {

	if ( ctx->isLocalCtx_init != TRUE )
		return;


	basisGaussNorm_local_s* local_ctx = &(ctx->local_ctx);

	my_basic_free_matrix ( local_ctx->time_mu );
	my_basic_free_matrix ( local_ctx->at );
	my_basic_free_matrix ( local_ctx->basis );
	my_free_vector ( local_ctx->basis_sum_vec, 1, 0 );
	my_basic_free_matrix ( local_ctx->basis_sum );
	my_basic_free_matrix ( local_ctx->time_mu_sigma );
	my_basic_free_matrix ( local_ctx-> basisD );
	my_free_vector ( local_ctx->basisD_sum_vec, 1, 0 );
	my_basic_free_matrix ( local_ctx->basisD_sum );
	my_basic_free_matrix ( local_ctx->basisD_n_a );
	my_basic_free_matrix ( local_ctx->basisD_n_b );
	my_basic_free_matrix ( local_ctx->basisD_n_a_min_b );
	my_basic_free_matrix ( local_ctx->tmp );
	my_basic_free_matrix ( local_ctx->basisDD );
	my_free_vector ( local_ctx->basisDD_sum_vec, 1, 0 );
	my_basic_free_matrix ( local_ctx->basisDD_sum );
	my_basic_free_matrix ( local_ctx->basisDD_n_a );
	my_basic_free_matrix ( local_ctx->basisDD_n_b1 );
	my_basic_free_matrix ( local_ctx->basisDD_n_b );
	my_basic_free_matrix ( local_ctx->basisDD_n_c1 );
	my_basic_free_matrix ( local_ctx->tmp1 );
	my_basic_free_matrix ( local_ctx->basisDD_n_c );
	my_basic_free_matrix ( local_ctx->basisDD_n_d );

	ctx->isLocalCtx_init = FALSE;

}



int basisGaussNorm_init ( Matrix time, Matrix mu, Matrix sigma, struct basis_gen_ctx_s* gen_ctx ) {

	int ok = TRUE;

	gen_ctx->bEval = basisGaussNorm;

	gen_ctx->basis_ctx = malloc( sizeof(basisGaussNorm_ctx_s) );

	if ( gen_ctx->basis_ctx == 0 )
		return FALSE;

	basisGaussNorm_ctx_s* ctx = gen_ctx->basis_ctx;

	if ( basisGaussNorm_initLocal ( time, mu, sigma, &(ctx->local_ctx) ) )
		ctx->isLocalCtx_init = TRUE;
	else {
		ctx->isLocalCtx_init = FALSE;
		return FALSE;
	}

	ctx->mu = my_matrix( 1, (int) mu[0][NR], 1, (int) mu[0][NC] );
	ctx->sigma = my_matrix( 1, (int) sigma[0][NR], 1, (int) sigma[0][NC] );

	int i,j;

	for ( i = 1; i <= (int) mu[0][NR]; ++i )
		for ( j = 1; j <= (int) mu[0][NC]; ++j )
			(ctx->mu)[i][j] = mu[i][j];

	for ( i = 1; i <= (int) sigma[0][NR]; ++i )
		for ( j = 1; j <= (int) sigma[0][NC]; ++j )
			(ctx->sigma)[i][j] = sigma[i][j];


	ok &= bsxfun_malloc ( ctx->local_ctx.basis, ctx->local_ctx.basis_sum, &(gen_ctx->basis_n) );
	ok &= bsxfun_malloc ( ctx->local_ctx.basisD_n_a_min_b, ctx->local_ctx.basis_sum, &(gen_ctx->basisD_n) );
	ok &= bsxfun_malloc ( ctx->local_ctx.basisDD_n_d, ctx->local_ctx.basis_sum, &(gen_ctx->basisDD_n) );


	return ok;
}



void basisGaussNorm_free ( struct basis_gen_ctx_s* gen_ctx ) {

	basisGaussNorm_ctx_s* ctx = gen_ctx->basis_ctx;

	basisGaussNorm_freeLocal ( ctx );

//	my_basic_free_matrix( ctx->basis_n);
//	my_basic_free_matrix( ctx->basisD_n);
//	my_basic_free_matrix( ctx->basisDD_n);

	my_basic_free_matrix( ctx->mu );
	my_basic_free_matrix( ctx->sigma );

	my_basic_free_matrix ( gen_ctx->basis_n );
	my_basic_free_matrix ( gen_ctx->basisD_n );
	my_basic_free_matrix ( gen_ctx->basisDD_n );

	free(gen_ctx->basis_ctx);

	gen_ctx->basis_ctx = 0;

}


static inline double f1 ( double a ){
	return exp( -0.5 * pow(a,2) );
}

static inline double f2 ( double a ){
	return 1.0 / a / sqrt(2*PI);
}

static inline double f3 ( double a ){
	return 1.0 / pow(a,2);
}

static inline double f4 ( double a ){
	return 2.0* pow(a,2);
}

static inline double f5 ( double a ){
	return 1.0 / pow(a,3);
}

static inline double my_inv ( double a ) {
	return 1.0 / a;
}

static inline double minus_el ( double a ) {
	return -a;
}

static inline double my_sqr_el ( double a ) {
	return pow(a,2);
}



int basisGaussNorm ( Matrix time, struct basis_gen_ctx_s* gen_ctx ) {


	Matrix basis_n   = gen_ctx->basis_n;
	Matrix basisD_n  = gen_ctx->basisD_n;
	Matrix basisDD_n = gen_ctx->basisDD_n;

	basisGaussNorm_ctx_s* ctx = gen_ctx->basis_ctx;


	if ( ctx->isLocalCtx_init == FALSE )
		return FALSE;


	basisGaussNorm_local_s* loc = &(ctx->local_ctx);

	int ok = TRUE;


	ok &= bsxfun ( my_Minus, time, ctx->mu, loc->time_mu, 0, 0 );   /* time_mu = bsxfun(@minus, time, mu ); */

	ok &= bsxfun ( my_Times, loc->time_mu, ctx->sigma, loc->at, 0, my_inv );      /* at = bsxfun(@times, time_mu, 1./sigma); */

	ok &= bsxfun ( my_Times, loc->at,  ctx->sigma, loc->basis, f1, f2  );    /* basis = bsxfun(@times, exp( -0.5 * at.^2 ), 1./sigma/sqrt(2*pi) ); */

	ok &= mat_sum_columns( loc->basis, loc->basis_sum_vec );       /* basis_sum = sum(basis,2); */
	vec_to_mat_op ( loc->basis_sum_vec, loc->basis_sum, 0 );


	ok &= bsxfun ( my_Times, loc->basis,  loc->basis_sum, basis_n, 0, my_inv );    /* basis_n = bsxfun(@times, basis, 1 ./ basis_sum); */


	ok &= bsxfun ( my_Times, loc->time_mu,  ctx->sigma, loc->time_mu_sigma, minus_el, f3 );    /* time_mu_sigma = bsxfun(@times, -time_mu, 1./(sigma.^2) ); */

	ok &= mat_dmult_normal_normal(loc->time_mu_sigma, loc->basis, loc->basisD);    /*basisD =  time_mu_sigma .* basis;*/

	ok &= mat_sum_columns( loc->basisD, loc->basisD_sum_vec );   /*basisD_sum = sum(basisD,2);*/
	vec_to_mat_op ( loc->basisD_sum_vec, loc->basisD_sum, 0 );


	ok &= bsxfun ( my_Times, loc->basisD,  loc->basis_sum, loc->basisD_n_a, 0, 0 );   /*basisD_n_a = bsxfun(@times, basisD, basis_sum);*/

	ok &= bsxfun ( my_Times, loc->basis,  loc->basisD_sum, loc->basisD_n_b, 0, 0 );   /*basisD_n_b = bsxfun(@times, basis, basisD_sum);*/

	mat_sub(loc->basisD_n_a, loc->basisD_n_b, loc->basisD_n_a_min_b); 	/* basisD_n_a - basisD_n_b */

	ok &= bsxfun ( my_Times, loc->basisD_n_a_min_b,  loc->basis_sum, basisD_n, 0, f3 );   /*basisD_n = bsxfun(@times, basisD_n_a - basisD_n_b, 1 ./(basis_sum.^2) );*/

	ok &= bsxfun ( my_Times, loc->basis,  ctx->sigma, loc->tmp, minus_el, f3 );   /*tmp =  bsxfun(@times,basis, -1./(sigma.^2) );*/

	ok &= mat_dmult_normal_normal(loc->time_mu_sigma, loc->basisD, loc->basisDD);
	ok &= mat_add(loc->basisDD, loc->tmp, loc->basisDD);  	 /*basisDD = tmp + time_mu_sigma .* basisD;*/


	ok &= mat_sum_columns( loc->basisDD, loc->basisDD_sum_vec );    /*basisDD_sum = sum(basisDD,2);*/
	vec_to_mat_op ( loc->basisDD_sum_vec, loc->basisDD_sum, 0 );


	ok &= bsxfun ( my_Times, loc->basisDD,  loc->basis_sum, loc->basisDD_n_a, 0, my_sqr_el ); 	/*basisDD_n_a = bsxfun(@times, basisDD, basis_sum.^2);*/


	ok &= bsxfun ( my_Times, loc->basisD,  loc->basis_sum, loc->basisDD_n_b1, 0, 0 );   /*basisDD_n_b1 = bsxfun(@times, basisD, basis_sum);*/


	ok &= bsxfun ( my_Times, loc->basisDD_n_b1,  loc->basisD_sum, loc->basisDD_n_b, 0, 0 );   /*basisDD_n_b = bsxfun(@times, basisDD_n_b1, basisD_sum);*/


	ok &= mat_dmult_normal_normal(loc->basis_sum, loc->basisDD_sum, loc->basisDD_n_c1);
	ok &= mat_equal_apply_math ( loc->basisD_sum, f4, loc->tmp1 );
	ok &= mat_sub( loc->tmp1, loc->basisDD_n_c1, loc->basisDD_n_c1);  /*basisDD_n_c1 =  2 * basisD_sum.^2 - basis_sum .* basisDD_sum;*/


	ok &= bsxfun ( my_Times, loc->basis,  loc->basisDD_n_c1, loc->basisDD_n_c, 0, 0 ); /*basisDD_n_c = bsxfun(@times, basis,  basisDD_n_c1);*/


	ok &= mat_mult_scalar(loc->basisDD_n_b, 2, loc->basisDD_n_d);
	ok &= mat_sub( loc->basisDD_n_a, loc->basisDD_n_d, loc->basisDD_n_d);
	ok &= mat_add( loc->basisDD_n_d, loc->basisDD_n_c, loc->basisDD_n_d);  	/*basisDD_n_d = basisDD_n_a - 2 .* basisDD_n_b + basisDD_n_c;*/


	ok &= bsxfun ( my_Times, loc->basisDD_n_d,  loc->basis_sum, basisDD_n, 0, f5 );  /*basisDD_n = bsxfun(@times, basisDD_n_d, 1 ./ basis_sum.^3);*/


	return ok;

}
