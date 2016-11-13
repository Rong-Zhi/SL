#ifndef CALCPHI_H_
#define CALCPHI_H_ 1

#include "utility.h"



int calcPhi ( Matrix basis, Matrix basisD, Matrix basisDD,
              int i, int numDim,
              Matrix Phi_t, Matrix Phi_t1, Matrix Phi_td );


#endif /* CALCPHI_H_ */
