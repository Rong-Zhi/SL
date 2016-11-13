#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"
#include <math.h>

#include "utility.h"
#include "ias_matrix_utilities.h"
#include "ias_matlab_utilities.h"

#include "pmps/basis/basisGeneric.h"
#include "pmps/basis/basisGaussNorm.h"




void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


	if ( nlhs == 0  )
		return;

	if ( nlhs > 3 ) {
		mexErrMsgTxt("At most three outputs...\n");
		return;
	}

	if ( nrhs != 3  ) {
		mexErrMsgTxt("It requires three input arguments.\n");
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


	double* a_raw = mxGetPr(prhs[0]);
	int a_rows = mxGetM(prhs[0]);
	int a_cols = mxGetN(prhs[0]);

	double* b_raw = mxGetPr(prhs[1]);
	int b_rows = mxGetM(prhs[1]);
	int b_cols = mxGetN(prhs[1]);

	double* c_raw = mxGetPr(prhs[2]);
	int c_rows = mxGetM(prhs[2]);
	int c_cols = mxGetN(prhs[2]);

	Matrix a = my_matrix(1, a_rows, 1, a_cols );
	Matrix b = my_matrix(1, b_rows, 1, b_cols );
	Matrix c = my_matrix(1, c_rows, 1, c_cols );

	int i,j;

	for ( i = 1; i <= a_rows; ++i )
		for ( j = 1; j <= a_cols; ++j )
			a[i][j] = a_raw[i-1 + a_rows*(j-1) ];

	for ( i = 1; i <= b_rows; ++i )
		for ( j = 1; j <= b_cols; ++j )
			b[i][j] = b_raw[i-1 + b_rows*(j-1) ];

	for ( i = 1; i <= c_rows; ++i )
		for ( j = 1; j <= c_cols; ++j )
			c[i][j] = c_raw[i-1 + c_rows*(j-1) ];



	struct basis_gen_ctx_s ctx;


	int ok = basisGaussNorm_init ( a,  b,  c, &ctx );


	ok &= ctx.bEval(a, &ctx );

	my_basic_free_matrix( a );
	my_basic_free_matrix( b );
	my_basic_free_matrix( c );

	if ( ok == FALSE ) {
		mexErrMsgTxt("basisGaussNorm c failed!\n");
		fflush(stdout);
		return;
	}


	plhs[0] = Matrix_to_mxArray( ctx.basis_n);

	if ( nlhs > 1 )
		plhs[1] = Matrix_to_mxArray(ctx.basisD_n);

	if ( nlhs > 2 )
		plhs[2] = Matrix_to_mxArray(ctx.basisDD_n);

	basisGaussNorm_free ( &ctx );

}
