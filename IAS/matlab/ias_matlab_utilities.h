#ifndef IAS_MATLAB_UTILITIES_H_
#define IAS_MATLAB_UTILITIES_H_ 1

#include "mex.h"
#include "utility.h"

/* Allocates a mxArray and store a Matrix */
mxArray* Matrix_to_mxArray (Matrix c);


/* Allocates and initializes a Matrix from a mxArray */
int mxArray_to_Matrix ( const mxArray* mx, Matrix* a);

#endif /* IAS_MATLAB_UTILITIES_H_ */
