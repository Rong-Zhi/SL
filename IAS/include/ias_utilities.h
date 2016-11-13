#ifndef __IAS_UTILITIES__
#define __IAS_UTILITIES__

#include "SL.h"
#include "utility.h"



int check_jointLimits(SL_DJstate *des_state, double th);
int set_toJointLimits(SL_DJstate *des_state, double th);

Vector getJinvWeights();

void cp_state_vec(SL_Cstate state,Vector x);
void cp_vec_state(SL_Cstate *state, Vector x);
void cp_sl_states(SL_DJstate* src, SL_DJstate* target);



#endif
