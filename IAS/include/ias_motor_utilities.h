#ifndef _IAS_MOTOR_UTILITIES_H_
#define _IAS_MOTOR_UTILITIES_H_ 1

#include "SL.h"
#include "SL_user.h"
#include "ias_common.h"

extern int firstGotoPos;
extern int inverseDynControl;
extern double kPos;
extern double kVel;

extern int reachedInitState;
extern int stepInTrajectory;
extern double t_ratio;

extern SL_OJstate rest_posture[N_DOFS+1];
extern SL_DJstate init_posture[N_DOFS+1];


void initMotorUtilities(int l_useInverseDynControl, double l_kPos, double l_kVel, SL_DJstate l_init_posture[N_DOFS+1], double error_thresh);
void setInverseDynamicsPDGains(double l_kPos, double l_kVel);


void initGotoPosMinJerkSL(double tau, int useVelocity);
int gotoPosMinJerk(double goal[N_DOFS+1]);
int gotoPosMinJerkSL(SL_DJstate goalState[N_DOFS+1]);

int gotoPos(double joints[N_DOFS+1]);

int doMotion(double joints[STEPLIMIT][N_DOFS+1], int step, int maxStep);
int doControl( double joints[STEPLIMIT][N_DOFS+1],  int step, int ctl_duration, int total_duration);

int ias_calculate_min_jerk_next_step (SL_DJstate *state, SL_DJstate *goal,double tau);


#endif // _IAS_MOTOR_UTILITIES_H_
