#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"

#include <math.h>
#include <string.h>

#include "utility.h"
#include "ias_matrix_utilities.h"
#include "pmps/bsxfun.h"




void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

	if ( nlhs == 0  )
		return;

	if ( nlhs != 1  ) {
		mexErrMsgTxt("Only one output should be provided.\n");
		return;
	}

	if ( nrhs != 3  ) {
		mexErrMsgTxt("It requires three input arguments.\n");
		return;
	}

	char buffer[128];

	if ( mxGetString( prhs[0], buffer, 128) ) {
		mexErrMsgTxt("Can't parse function name.\n");
		return;
	}

	bsxfunOp op = 0;

	if ( strcmp(buffer, "plus") == 0 )
		op = my_Plus;
	else if ( strcmp(buffer, "times") == 0 )
		op = my_Times;
	else {
		mexErrMsgTxt("Function not supported.\n");
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


	double* a_raw = mxGetPr(prhs[1]);
	int a_rows = mxGetM(prhs[1]);
	int a_cols = mxGetN(prhs[1]);

	double* b_raw = mxGetPr(prhs[2]);
	int b_rows = mxGetM(prhs[2]);
	int b_cols = mxGetN(prhs[2]);

	Matrix a = my_matrix(1, a_rows, 1, a_cols );
	Matrix b = my_matrix(1, b_rows, 1, b_cols );

	int i,j;

	for ( i = 1; i <= a_rows; ++i )
		for ( j = 1; j <= a_cols; ++j )
			a[i][j] = a_raw[i-1 + a_rows*(j-1) ];

	for ( i = 1; i <= b_rows; ++i )
		for ( j = 1; j <= b_cols; ++j )
			b[i][j] = b_raw[i-1 + b_rows*(j-1) ];


	Matrix c = 0;

	if ( ! bsxfun_malloc ( a, b, &c ) ) {
		mexErrMsgTxt("bsxfun malloc c failed!.\n");
		return;
	}

	if ( ! bsxfun ( op, a, b, c, 0, 0 ) ) {
		mexErrMsgTxt("bsxfun c failed!.\n");
		return;
	}

	int c_rows = (int) c[0][NR];
	int c_cols = (int) c[0][NC];

	plhs[0] = mxCreateDoubleMatrix(c_rows, c_cols, mxREAL);
	double* c_raw = mxGetPr(plhs[0]);

	for ( i = 1; i <= c_rows; ++i )
		for ( j = 1; j <= c_cols; ++j )
			c_raw[i-1 + c_rows*(j-1) ] = c[i][j];

	my_basic_free_matrix( c );

}
