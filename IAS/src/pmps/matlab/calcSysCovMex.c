#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"
#include <math.h>

#include "utility.h"
#include "ias_matlab_utilities.h"
#include "ias_matrix_utilities.h"
#include "pmps/calcSysCov.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


	if ( nlhs == 0  )
		return;

	if ( nlhs > 6 ) {
		mexErrMsgTxt("At most six outputs...\n");
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


	calcSysCovCtx_s ctx;

	ok &= calcSysCovBasic_init(numDim, &ctx);


	Matrix time = my_matrix(1,2,1,1);

	time[1][1] = phase[idx][1];
	time[2][1] = phase[idx+1][1];


//	printf("sdfsdafsa\n");
//	fflush(stdout);
//	print_mat("time",time);
//	printf("\n");
//	fflush(stdout);



	ok &= calcSysCovBasic_addBasisGauss ( &ctx, time, w_mu, w_cov, mu, sigma );



	if ( ok == FALSE ) {
		mexErrMsgTxt("Failed to initialize!\n");
		return;
	}



	ok &= calcSysCovBasic(time, &ctx);



	if ( ok == FALSE ) {
		mexErrMsgTxt("calcSysCovBasic  c failed!\n");
		fflush(stdout);
		return;
	}

	plhs[0] = Matrix_to_mxArray(ctx.mu_x);
	plhs[1] = Matrix_to_mxArray(ctx.mu_xd);

	plhs[2] = Matrix_to_mxArray(ctx.Sigma_t);
	plhs[3] = Matrix_to_mxArray(ctx.Sigma_t1);
	plhs[4] = Matrix_to_mxArray(ctx.Sigma_t_t1);
	plhs[5] = Matrix_to_mxArray(ctx.Sigma_td_half);



}
