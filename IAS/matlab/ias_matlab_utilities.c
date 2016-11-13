#include "ias_matlab_utilities.h"

mxArray* Matrix_to_mxArray (Matrix c) {

	int c_rows = (int) c[0][NR];
	int c_cols = (int) c[0][NC];

	int i,j;

	mxArray* mxAr = mxCreateDoubleMatrix(c_rows, c_cols, mxREAL);
	double* c_raw = mxGetPr(mxAr);

	for ( i = 1; i <= c_rows; ++i )
		for ( j = 1; j <= c_cols; ++j )
			c_raw[i-1 + c_rows*(j-1) ] = c[i][j];

	return mxAr;

}


int mxArray_to_Matrix ( const mxArray* mx, Matrix* a) {

	if ( ! mxIsDouble( mx ) ) {
		mexErrMsgTxt("Argument should be a double, a double vector, or a double Matrix.\n");
		return FALSE;
	}

	double* a_raw = mxGetPr( mx );
	int a_rows    = mxGetM ( mx );
	int a_cols    = mxGetN ( mx );

	*a = my_matrix(1, a_rows, 1, a_cols );

	int i,j;

	for ( i = 1; i <= a_rows; ++i )
		for ( j = 1; j <= a_cols; ++j )
			(*a)[i][j] = a_raw[i-1 + a_rows*(j-1) ];

	return TRUE;

}
