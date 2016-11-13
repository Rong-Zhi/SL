#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"
#include <math.h>

#include "utility.h"
#include "ias_matrix_utilities.h"
#include "ias_matlab_utilities.h"

#include "pmps/basis/basisGeneric.h"
#include "pmps/basis/basisVonMises.h"




void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


	if ( nlhs == 0  )
		return;

	if ( nlhs > 3 ) {
		mexErrMsgTxt("At most three outputs...\n");
		return;
	}

	if ( nrhs != 4  ) {
		mexErrMsgTxt("It requires four input arguments.\n");
		return;
	}

	if ( ! mxIsDouble(prhs[0]) ) {
		mexErrMsgTxt("First argument should be a double, a double vector, or a double Matrix.\n");
		return;
	}
	if ( ! mxIsDouble(prhs[1]) ) {
		mexErrMsgTxt("Second argument should be a double, a double vector, or a double Matrix.\n");
		return;
	}
	if ( ! mxIsDouble(prhs[2]) ) {
		mexErrMsgTxt("Third argument should be a double, a double vector, or a double Matrix.\n");
		return;
	}



	int ok = TRUE;


	Matrix phase, mu;
	double k = 0;
	double f = 0;

	ok &= mxArray_to_Matrix ( prhs[0], &phase);
	ok &= mxArray_to_Matrix ( prhs[1], &mu);


	k = *mxGetPr(prhs[2]);
	f = *mxGetPr(prhs[3]);

	if ( ok == FALSE ) {
		mexErrMsgTxt("Failed to parse arguments!\n");
		return;
	}



	struct basis_gen_ctx_s ctx;

	ok &= basisVonMises_init ( phase, mu, k, f, &ctx);

	my_basic_free_matrix( mu );

	if ( ok == FALSE ) {
		mexErrMsgTxt("Failed to init!\n");
		return;
	}

	ok &= ctx.bEval(phase, &ctx );


	if ( ok == FALSE ) {
		mexErrMsgTxt("basisGaussNorm c failed!\n");
		return;
	}

	my_basic_free_matrix( phase );





	plhs[0] = Matrix_to_mxArray( ctx.basis_n);

	if ( nlhs > 1 )
		plhs[1] = Matrix_to_mxArray ( ctx.basisD_n);

	if ( nlhs > 2 )
		plhs[2] = Matrix_to_mxArray(ctx.basisDD_n);


	 basisVonMises_free( &ctx );


}
