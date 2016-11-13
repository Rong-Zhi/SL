#include "mex.h"

#include "SL_user.h"
#include "SL_episodic_communication.h"

void mexFunction(int nlhs, mxArray *plhs[], 
                 int nrhs, const mxArray *prhs[])
{

	plhs[0] = mxCreateDoubleScalar(N_DOFS);
 	plhs[1] = mxCreateDoubleScalar(N_DOFS_SHM);
	plhs[2] = mxCreateDoubleScalar(STEPLIMITEPISODE);
	
}
