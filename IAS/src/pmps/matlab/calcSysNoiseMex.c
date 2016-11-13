#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"
#include <math.h>

#include "utility.h"
#include "ias_matlab_utilities.h"
#include "ias_matrix_utilities.h"
#include "pmps/calcSysNoise.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


	if ( nlhs == 0  )
		return;

	if ( nlhs > 1 ) {
		mexErrMsgTxt("At most one output...\n");
		return;
	}

	if ( nrhs != 4  ) {
		mexErrMsgTxt("It requires four input arguments.\n");
		return;
	}

	Matrix Sigma_t, Sigma_t1, Sigma_t_t1;

	int ok = TRUE;

	ok &= mxArray_to_Matrix ( prhs[0], &Sigma_t);
	ok &= mxArray_to_Matrix ( prhs[1], &Sigma_t1);
	ok &= mxArray_to_Matrix ( prhs[2], &Sigma_t_t1);

	if ( ok == FALSE ) {
		mexErrMsgTxt("Failed to parse arguments!\n");
		return;
	}

	int ndim = (int) Sigma_t[0][NR] / 2;

	Matrix Sigma_x_out = my_matrix ( 1, (int) Sigma_t[0][NR], 1, (int) Sigma_t[0][NC] );

	calcSysNoise_local_s ctx;
	ok &= calcSysNoise_initLocal ( ndim, &ctx);

	ok &= calcSysNoise( Sigma_t, Sigma_t1, Sigma_t_t1, Sigma_x_out, &ctx );

	calcSysNoise_freeLocal ( &ctx);

	my_basic_free_matrix ( Sigma_t );
	my_basic_free_matrix ( Sigma_t1 );
	my_basic_free_matrix ( Sigma_t_t1 );

	if ( ok == FALSE ) {
		mexErrMsgTxt("calcSysNoise c failed!\n");
		return;
	}

	plhs[0] = Matrix_to_mxArray(Sigma_x_out);

	my_basic_free_matrix ( Sigma_x_out );

}
