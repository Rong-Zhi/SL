#ifndef BSXFUN_H_
#define BSXFUN_H_ 1

#include "utility.h"

typedef double (*bsxfunOp) (double, double);
typedef double (*bsxfunElemOp) (double);

double my_Plus   (double a, double b);
double my_Minus  (double a, double b);
double my_Times  (double a, double b);


int bsxfun_malloc ( Matrix a_in, Matrix b_in, Matrix* c_out );

int bsxfun ( bsxfunOp op, Matrix a_in, Matrix b_in,   /* input arguments */
             Matrix c_out, 							  /* output arguments */
             bsxfunElemOp a_op, bsxfunElemOp b_op );  /* optional arguments, can be NULL */



#endif /* BSXFUN_H_ */
