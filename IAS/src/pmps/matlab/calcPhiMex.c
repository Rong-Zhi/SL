#ifndef MATLAB
#define MATLAB
#endif

#include "mex.h"
#include <math.h>


#include "utility.h"
#include "ias_matlab_utilities.h"
#include "ias_matrix_utilities.h"
#include "pmps/calcPhi.h"





void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {


	if ( nlhs == 0  )
		return;

	if ( nlhs > 3 ) {
		mexErrMsgTxt("At most three outputs...\n");
		return;
	}

	if ( nrhs != 5  ) {
		mexErrMsgTxt("It requires five input arguments.\n");
		return;
	}

	int ok = TRUE;


	Matrix basis, basisD, basisDD;
	int i = 0, numDim = 0;

	ok &= mxArray_to_Matrix ( prhs[0], &basis);
	ok &= mxArray_to_Matrix ( prhs[1], &basisD);
	ok &= mxArray_to_Matrix ( prhs[2], &basisDD);

	i      = *mxGetPr(prhs[3]);
	numDim = *mxGetPr(prhs[4]);

	if ( ok == FALSE ) {
		mexErrMsgTxt("Failed to parse arguments!\n");
		return;
	}

//		Phi_t_mDim  = zeros ( numBasis*numDim, 2 * numDim );
//		Phi_t1_mDim = zeros ( numBasis*numDim, 2 * numDim );
//		Phi_td_mDim = zeros ( numBasis*numDim, 2 * numDim );

	int numBasis = (int) basis[0][NC];

	Matrix Phi_t  = my_matrix ( 1, numBasis*numDim, 1, 2 * numDim );
	Matrix Phi_t1 = my_matrix ( 1, numBasis*numDim, 1, 2 * numDim );
	Matrix Phi_td = my_matrix ( 1, numBasis*numDim, 1, 2 * numDim );


	ok &= calcPhi ( basis, basisD, basisDD,
	                i, numDim,
	                Phi_t, Phi_t1, Phi_td );


	my_basic_free_matrix ( basis );
	my_basic_free_matrix ( basisD );
	my_basic_free_matrix ( basisDD );


	if ( ok == FALSE ) {
		mexErrMsgTxt("calcGains c failed!\n");
		return;
	}

	plhs[0] = Matrix_to_mxArray(Phi_t);

	my_basic_free_matrix ( Phi_t );


	if ( nlhs > 1 )
		plhs[1] = Matrix_to_mxArray(Phi_t1);

	my_basic_free_matrix ( Phi_t1 );

	if ( nlhs > 2 )
		plhs[2] = Matrix_to_mxArray(Phi_td);

	my_basic_free_matrix ( Phi_td );


}
