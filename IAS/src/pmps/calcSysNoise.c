#include "pmps/calcSysNoise.h"


#include "ias_matrix_utilities.h"


int calcSysNoise_initLocal ( int ndim, calcSysNoise_local_s* ctx) {

	if ( ctx->isInit == TRUE )
		return FALSE;

	ctx->isInit = TRUE;

//	ctx->Sigma_t_tmp = my_matrix( 1, ndim*nBasis, 1, 2*ndim  );
//	ctx->Sigma_t = my_matrix( 1, 2*ndim, 1, (int) (ctx->Sigma_t_tmp)[0][NC]  );
//	ctx->Sigma_t1_tmp = my_matrix( 1, ndim*nBasis, 1, 2*ndim  );
//	ctx->Sigma_t1 = my_matrix( 1, 2*ndim, 1, (int) (ctx->Sigma_t1_tmp)[0][NC]  );
//	ctx->Sigma_t_t1_tmp = my_matrix( 1, ndim*nBasis, 1, 2*ndim  );
//	ctx->Sigma_t_t1 = my_matrix( 1, 2*ndim, 1, (int) (ctx->Sigma_t_t1_tmp)[0][NC]  );
	ctx->Sigma_t_inv = my_matrix( 1, 2*ndim, 1,  2*ndim  );
	ctx->Sigma_x_tmp = my_matrix( 1, 2*ndim, 1,  2*ndim  );

	return TRUE;

}

void calcSysNoise_freeLocal ( calcSysNoise_local_s* ctx) {

	if ( ! ctx->isInit )
			return;

//	my_basic_free_matrix ( ctx->Sigma_t_tmp );
//	my_basic_free_matrix ( ctx->Sigma_t );
//	my_basic_free_matrix ( ctx->Sigma_t1_tmp );
//	my_basic_free_matrix ( ctx->Sigma_t1 );
//	my_basic_free_matrix ( ctx->Sigma_t_t1_tmp );
//	my_basic_free_matrix ( ctx->Sigma_t_t1 );
	my_basic_free_matrix ( ctx->Sigma_t_inv );
	my_basic_free_matrix ( ctx->Sigma_x_tmp );

}


int calcSysNoise( Matrix Sigma_t, Matrix  Sigma_t1, Matrix  Sigma_t_t1, Matrix Sigma_x_out, calcSysNoise_local_s* ctx ) {

	if ( ctx->isInit != TRUE )
		return FALSE;


	int ok = TRUE;

	ok &= my_inv_ludcmp( Sigma_t, (int) Sigma_t[0][NR], ctx->Sigma_t_inv);


	ok &= mat_mult_transpose_normal( Sigma_t_t1, ctx->Sigma_t_inv, ctx->Sigma_x_tmp );

	ok &= mat_mult( ctx->Sigma_x_tmp, Sigma_t_t1, Sigma_x_out);
	ok &= mat_sub(  Sigma_t1, Sigma_x_out, Sigma_x_out );  /* Sigma_u = Sigma_t1 - Sigma_t_t1' / Sigma_t * Sigma_t_t1; */


	return ok;

}


//int calcSysNoise( Matrix Phi_t, Matrix  Phi_t1, Matrix  w_cov, Matrix Sigma_x_est, Matrix Sigma_x_out, calcSysNoise_local_s* ctx ) {
//
//	if ( ctx->isInit != TRUE )
//		return FALSE;
//
//
//	int ok = TRUE;
//
//	ok &= mat_mult(w_cov,Phi_t, ctx->Sigma_t_tmp);
//	ok &= mat_mult_transpose_normal(Phi_t,ctx->Sigma_t_tmp,ctx->Sigma_t);
//	ok &= mat_add(ctx->Sigma_t,Sigma_x_est,ctx->Sigma_t);                    /* Sigma_t = Phi_t' * w_cov * Phi_t + Sigma_x; */
//
//
//	ok &= mat_mult(w_cov,Phi_t1,ctx->Sigma_t1_tmp);
//	ok &= mat_mult_transpose_normal(Phi_t1,ctx->Sigma_t1_tmp,ctx->Sigma_t1);
//	ok &= mat_add(ctx->Sigma_t1,Sigma_x_est,ctx->Sigma_t1);           /* Sigma_t1 = Phi_t1' * w_cov * Phi_t1 + Sigma_x; */
//
//
//
//
//	ok &= mat_mult(w_cov,Phi_t1,ctx->Sigma_t_t1_tmp);
//	ok &= mat_mult_transpose_normal(Phi_t,ctx->Sigma_t_t1_tmp,ctx->Sigma_t_t1);    /* Sigma_t_t1 = Phi_t' * w_cov * Phi_t1; */
//
//
//
//	ok &= my_inv_ludcmp(ctx->Sigma_t, (int) (ctx->Sigma_t)[0][NR], ctx->Sigma_t_inv);
//
//
//	ok &= mat_mult_transpose_normal(ctx->Sigma_t_t1,ctx->Sigma_t_inv,ctx->Sigma_x_tmp);
//
//	ok &= mat_mult(ctx->Sigma_x_tmp,ctx->Sigma_t_t1,Sigma_x_out);
//	ok &= mat_sub(ctx->Sigma_t1,Sigma_x_out,Sigma_x_out);  /* Sigma_u = Sigma_t1 - Sigma_t_t1' / Sigma_t * Sigma_t_t1; */
//
//
//	return ok;
//
//}
