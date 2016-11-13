#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"
#include <math.h>

#include "utility.h"
#include "ias_matrix_utilities.h"
#include "ias_matlab_utilities.h"
#include "pmps/calcGains.h"


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


	if ( nlhs == 0  )
		return;

	if ( nlhs > 2 ) {
		mexErrMsgTxt("At most two outputs...\n");
		return;
	}

	if ( nrhs != 10  ) {
		mexErrMsgTxt("It requires ten input arguments.\n");
		return;
	}



	int ok = TRUE;


//	Matrix w_cov;
//	Matrix Phi_t, Phi_td;
	Matrix Sigma_t, Sigma_tD_half;
	Matrix Sigma_x, Sigma_u;
	Matrix mu_x, mu_xd;
	Matrix A, B, c;
	double dt;


//	ok &= mxArray_to_Matrix ( prhs[0], &w_cov);
//	ok &= mxArray_to_Matrix ( prhs[1], &Phi_t);
//	ok &= mxArray_to_Matrix ( prhs[2], &Phi_td);
	ok &= mxArray_to_Matrix ( prhs[0], &Sigma_t);
	ok &= mxArray_to_Matrix ( prhs[1], &Sigma_tD_half);
	ok &= mxArray_to_Matrix ( prhs[2], &Sigma_x);
	ok &= mxArray_to_Matrix ( prhs[3], &Sigma_u);
	ok &= mxArray_to_Matrix ( prhs[4], &mu_x);
	ok &= mxArray_to_Matrix ( prhs[5], &mu_xd);
	ok &= mxArray_to_Matrix ( prhs[6], &A);
	ok &= mxArray_to_Matrix ( prhs[7], &B);
	ok &= mxArray_to_Matrix ( prhs[8], &c);

	dt = *mxGetPr(prhs[9]);

	if ( ok == FALSE ) {
		mexErrMsgTxt("Failed to parse arguments!\n");
		return;
	}


	Matrix K = my_matrix ( 1, ((int) Sigma_u[0][NR]) / 2, 1, (int) Sigma_u[0][NC] );
	Matrix k = my_matrix ( 1, ((int) Sigma_u[0][NR]) / 2, 1, 1 );


	calcGains_local_s local_ctx;
	calcGain_initLocal ( (int) Sigma_t[0][NC]/2, &local_ctx );


	ok &= calcGains( Sigma_t, Sigma_tD_half, Sigma_x, Sigma_u, mu_x, mu_xd, A, B, c, dt, K, k, &local_ctx) ;


//	my_basic_free_matrix ( w_cov );
//	my_basic_free_matrix ( Phi_t );
//	my_basic_free_matrix ( Phi_td );
	my_basic_free_matrix ( Sigma_t );
	my_basic_free_matrix ( Sigma_tD_half );
	my_basic_free_matrix ( Sigma_x );
	my_basic_free_matrix ( Sigma_u );
	my_basic_free_matrix ( mu_x );
	my_basic_free_matrix ( mu_xd );
	my_basic_free_matrix ( A );
	my_basic_free_matrix ( B );
	my_basic_free_matrix ( c );

	calcGain_freeLocal ( &local_ctx );


	if ( ok == FALSE ) {
		mexErrMsgTxt("calcGains c failed!\n");
		return;
	}

	plhs[0] = Matrix_to_mxArray(K);

	my_basic_free_matrix ( K );


	if ( nlhs > 1 )
		plhs[1] = Matrix_to_mxArray(k);

	my_basic_free_matrix ( k );


}
