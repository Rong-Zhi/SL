// SL general includes of system headers
#include "SL_system_headers.h"

#include "SL.h"
#include "SL_common.h"
#include "SL_user.h"
#include "SL_dynamics.h"
#include "utility.h"

#include <math.h>

void spline5th(SL_DJstate *joint, SL_DJstate *start, SL_DJstate *stop, double t, double T);
void spline5thvia(SL_DJstate *joint, SL_DJstate *start, SL_DJstate *stop, SL_DJstate *viapoint, SL_endeff *endeff, SL_Cstate predVHP_I, double t, double T1, double T2, int update, int torque);
void s5via(SL_DJstate *joint, SL_DJstate *start, SL_DJstate *stop, SL_DJstate *viapoint, double t, double T1, double T2, int update);
static void s5via_torque(SL_DJstate *joint, SL_DJstate *start, SL_DJstate *stop, SL_DJstate *viapoint, SL_endeff  *endeff, SL_Cstate predVHP_I, double t, double T1, double T2, int update);
