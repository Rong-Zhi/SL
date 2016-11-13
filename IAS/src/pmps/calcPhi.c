#include "pmps/calcPhi.h"


int calcPhi ( Matrix basis, Matrix basisD, Matrix basisDD,
              int i, int numDim,
              Matrix Phi_t, Matrix Phi_t1, Matrix Phi_td )
{

	int ok = TRUE;


	int numBasis = (int) basis[0][NC];  /* numBasis = size(basis,2); */


	mat_zero ( Phi_t );     /*	Phi_t_mDim  = zeros ( numBasis*numDim, 2 * numDim ); */
	mat_zero ( Phi_t1 );    /*	Phi_t1_mDim = zeros ( numBasis*numDim, 2 * numDim ); */
	mat_zero ( Phi_td );    /*	Phi_td_mDim = zeros ( numBasis*numDim, 2 * numDim ); */

	int k,l;

	for ( k = 1; k <= numDim; ++k ) {

	    int idx_i_start = (k-1) * numBasis;

	    for ( l = 1; l <= numBasis; ++l ) {

	    	Phi_t[idx_i_start+l][k] = basis[i][l];
	    	Phi_t[idx_i_start+l][k+numDim] = basisD[i][l];

	    	Phi_t1[idx_i_start+l][k] = basis[i+1][l];
	    	Phi_t1[idx_i_start+l][k+numDim] = basisD[i+1][l];

	    	Phi_td[idx_i_start+l][k] = basisD[i][l];
	    	Phi_td[idx_i_start+l][k+numDim] = basisDD[i][l];

	    }

	}

	return ok;

}
