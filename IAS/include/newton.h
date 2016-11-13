#ifndef _NEWTON_H_INC_
#define _NEWTON_H_INC_ 0

#include "utility.h"

typedef void (*newFuncP_t) ( int, Vector, Vector, Vector );
typedef void (*newFunc_J_P_t) ( Vector, Matrix, Vector );

int newt ( 	Vector x,
			Vector param,
			int dim,
			int *check,
			newFuncP_t vecfunc,
			newFunc_J_P_t bvjac );

#endif //_NEWTON_H_INC_
