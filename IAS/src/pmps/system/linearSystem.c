#include "pmps/system/linearSystem.h"

int linearSystemMat ( Vector q,
                      Matrix A, Matrix B, Matrix c ) {

	(void)(q);

	int i;

	int dim = (int) A[0][NR] / 2;  /* n = size(q,1)/2; */

	mat_zero( A );

	for ( i = 1; i <= dim ; ++i )
		A[i][dim+i] = 1.0;

/*	A = [  zeros(n), eye(n);
	      zeros(n,2*n);
	    ];
*/


	mat_zero( B );

	for ( i = 1; i <= dim ; ++i )
		B[dim+i][i] = 1.0;

/*
	B = [ zeros(n,n);
	      eye(n,n);
	    ];
*/

	mat_zero( c );   /* c = zeros(2*n,1); */

	return TRUE;
}
