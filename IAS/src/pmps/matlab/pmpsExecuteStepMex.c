#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"
#include <math.h>

#include "utility.h"
#include "ias_matlab_utilities.h"
#include "ias_matrix_utilities.h"
#include "pmps/pmps.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


	if ( nlhs == 0  )
		return;

	if ( nlhs > 2 ) {
		mexErrMsgTxt("At most two outputs...\n");
		return;
	}

	if ( nrhs != 9  ) {
		mexErrMsgTxt("It requires nine input arguments.\n");
		return;
	}


	int ok = TRUE;

	Matrix phase, mu, sigma;
	Matrix w_mu, w_cov;



//	phase, properties.basis_mu, properties.basis_sigma,...
//	                                   w_mu,  w_cov, ...
//	                                   numDim,  numBasis, dt, i


	ok &= mxArray_to_Matrix ( prhs[0], &phase);
	ok &= mxArray_to_Matrix ( prhs[1], &mu);
	ok &= mxArray_to_Matrix ( prhs[2], &sigma);

	ok &= mxArray_to_Matrix ( prhs[3], &w_mu);
	ok &= mxArray_to_Matrix ( prhs[4], &w_cov);


	int numDim   = (int) *mxGetPr(prhs[5]);
	int numBasis = (int) *mxGetPr(prhs[6]);
	double dt    =  	 *mxGetPr(prhs[7]);
	int idx      = (int) *mxGetPr(prhs[8]);


	if ( ok == FALSE ) {
		mexErrMsgTxt("Failed to parse arguments!\n");
		return;
	}


	pmps_ctx ctx;

	ok &= pmps_init ( numDim, dt, &ctx );


	Matrix time = my_matrix(1,2,1,1);

	time[1][1] = phase[idx][1];
	time[2][1] = phase[idx+1][1];


//	printf("sdfsdafsa\n");
//	fflush(stdout);
//	print_mat("time",time);
//	printf("\n");
//	fflush(stdout);


	ok &= calcSysCovBasic_addBasisGauss ( &(ctx.calcSysCov_ctx), time, w_mu, w_cov, mu, sigma );

	ctx->calcSysCovF = calcSysCovBasic;


	if ( ok == FALSE ) {
		mexErrMsgTxt("Failed to initialize!\n");
		return;
	}


	Matrix K_out = my_matrix ( 1, numDim, 1, 2*numDim );
	Matrix k_out = my_matrix ( 1, numDim, 1, 1 );


	ok &= pmps_execute_step ( &ctx, time, K_out, k_out );



//	my_basic_free_matrix ( basis);
//	my_basic_free_matrix ( basisD);
//	my_basic_free_matrix ( basisDD);
//
//	my_basic_free_matrix ( w_mu);
//	my_basic_free_matrix ( w_cov);
//
//	ok &= pmps_free ( &ctx);


	if ( ok == FALSE ) {
		mexErrMsgTxt("pmpsExecute step c failed!\n");
		fflush(stdout);
		return;
	}

	plhs[0] = Matrix_to_mxArray(K_out);

//	my_basic_free_matrix ( K_out );

	if ( nlhs > 1 )
		plhs[1] = Matrix_to_mxArray(k_out);

//	my_basic_free_matrix ( k_out );

}
