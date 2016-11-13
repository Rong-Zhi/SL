#ifndef BASISGENERIC_H_
#define BASISGENERIC_H_ 1


#include "utility.h"

struct basis_gen_ctx_s;


typedef int (*basisEvalFunc) ( Matrix phase, struct basis_gen_ctx_s* ctx );


struct basis_gen_ctx_s{

	Matrix basis_n, basisD_n, basisDD_n;
	basisEvalFunc bEval;
	void* basis_ctx;

};



#endif /* BASISGENERIC_H_ */
