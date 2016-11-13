#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"
#include <math.h>

#include "utility.h"
#include "ias_matrix_utilities.h"
#include "ias_matlab_utilities.h"

#include "pmps/system/linearSystem.h"




void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


	if ( nlhs == 0  )
		return;

	if ( nlhs > 3 ) {
		mexErrMsgTxt("At most three outputs...\n");
		return;
	}

	if ( nrhs != 1  ) {
		mexErrMsgTxt("It requires one input argument.\n");
		return;
	}


	int a_rows = mxGetM(prhs[0]);
	int a_cols = mxGetN(prhs[0]);

	int dim = (a_rows > a_cols ? a_rows : a_cols) / 2;

	Matrix A = my_matrix(1, 2*dim, 1, 2*dim );
	Matrix B = my_matrix(1, 2*dim, 1, dim );
	Matrix c = my_matrix(1, 2*dim, 1, 1 );


	linearSystemMat ( 0, A, B, c );


	plhs[0] = Matrix_to_mxArray( A );

	if ( nlhs > 1 )
		plhs[1] = Matrix_to_mxArray( B );

	if ( nlhs > 2 )
		plhs[2] = Matrix_to_mxArray( c);

	my_basic_free_matrix( A );
	my_basic_free_matrix( B );
	my_basic_free_matrix( c );

}
