#include "ias_motor_utilities.h"

#include "ias_utilities.h"
#include <math.h>

#include "SL_task_servo.h"

SL_OJstate rest_posture[N_DOFS+1];
SL_DJstate init_posture[N_DOFS+1];
 
double maxAcceleration = 2;
double tolerance 	= 0.02;
static int reachedGoal = 0;

int inverseDynControl = -1;
double kPos = 80;
double kVel = 20;

int reachedInitState = 0;
int stepInTrajectory = 0;
static int counter = 0;
static double tau;
static SL_DJstate currState[N_DOFS+1];
static double maxJointVel[N_DOFS + 1];
double t_ratio = 1.0;

void setMaxJointVelocitiesScalar(double val)
{
	int i = 0;
	for (i = 1; i <= N_DOFS; i++)
	{
		maxJointVel[i] = val;
	}
}

void setMaxJointVelocitiesVector(double val[N_DOFS + 1])
{
	int i = 0;
	for (i = 1; i <= N_DOFS; i++)
	{
		maxJointVel[i] = val[i];
	}
}

void setTimeRatio(double l_ratio)
{
	t_ratio = l_ratio;
}


void initMotorUtilities(int l_useInverseDynControl, double l_kPos, double l_kVel, SL_DJstate l_init_posture[N_DOFS+1], double error_thresh)
{
	int i = 0;
	for (i = 1; i <= N_DOFS; i++)
	{
		init_posture[i].th = l_init_posture[i].th;
		init_posture[i].thd = 0;
		init_posture[i].thdd = 0;
		
		rest_posture[i].th = l_init_posture[i].th;
	}
	
	kPos = l_kPos;
	kVel = l_kVel;
	inverseDynControl = l_useInverseDynControl;
	tolerance = error_thresh;
	stepInTrajectory = 0;
	setMaxJointVelocitiesScalar(100.0);
}

void setInverseDynamicsPDGains(double l_kPos, double l_kVel)
{
	kPos = l_kPos;
	kVel = l_kVel;
}

void initGotoPosMinJerkSLDesState(double l_tau, int useVelocity)
{
	int i = 0;
	for (i = 1; i <= N_DOFS; i ++)
	{
		currState[i].th 	= joint_des_state[i].th;
		if (useVelocity)
		{
			currState[i].thd 	= joint_des_state[i].thd;
		}
		else
		{
			currState[i].thd 	= 0.f;
		}
		currState[i].thdd 	= 0.f;
	}

	counter = 0;
	tau = l_tau;
	firstGotoPos = 0;
}

int gotoPosMinJerk(double goal[N_DOFS+1])
{
	SL_DJstate goalState[N_DOFS+1];
	int i = 0;
	FOR(i, N_DOFS )
	{
		goalState[i].th 	= goal[i];
		goalState[i].thd 	= 0.f;
		goalState[i].thdd 	= 0.f;
	}
	return gotoPosMinJerkSL(goalState);
}


void initGotoPosMinJerkSL(double l_tau, int useVelocity)
{
	int i = 0;
	for (i = 1; i <= N_DOFS; i ++)
	{
		currState[i].th 	= joint_state[i].th;
		if (useVelocity)
		{
			currState[i].thd 	= joint_state[i].thd;
		}
		else
		{
			currState[i].thd 	= 0.f;
		}
		currState[i].thdd 	= 0.f;
	}

	counter = 0;
	tau = l_tau;
	firstGotoPos = 0;
	stepInTrajectory = 0;
}




int gotoPosMinJerkSL(SL_DJstate goalState[N_DOFS+1])
{
	int i = 0;
	if (inverseDynControl == -1)
	{
		printf("Please Initialize the motor utilities!\n");
		freeze();
	}
	
	if (firstGotoPos == 1)
	{
		initGotoPosMinJerkSL(1.0 / t_ratio, FALSE);
//		printf("First GotoStartMinJerk %d, Tau %f, %f %f %f %f %f %f %f \n", counter, tau, currState[1].th, 
//				currState[2].th, currState[3].th, currState[4].th, currState[5].th, currState[6].th, currState[7].th);
	
	}

	// kinematics follow min merk
	  if (!ias_calculate_min_jerk_next_step (currState, goalState,tau+1e-15))
	  {
	    printf("This should never happen\n");
	    return TRUE;
	  }
//	  printf("GotoStartMinJerk %d, Tau %f, %f %f %f %f %f %f %f \n", counter, tau, currState[1].th, 
//				currState[2].th, currState[3].th, currState[4].th, currState[5].th, currState[6].th, currState[7].th);
		
			
	  tau -= 1./((double)task_servo_rate);
	  if(tau < 1./((double)task_servo_rate) )
	  {
		  tau = 1./((double)task_servo_rate );
	  }

//	  printf("Tau : %f\n", tau);
	  FOR(i, N_DOFS )
	 {
//		  printf("joint %d : desState %f realState %f realVel %f, realTorque %f, fbTorque %f, ffTorque %f\n"
//				  , i, currState[i].th,
//				  joint_state[i].th, joint_state[i].thd, joint_state[i].u, joint_state[i].ufb, joint_des_state[i].uff );
		  joint_des_state[i].th 	= currState[i].th;
		  joint_des_state[i].thd 	= currState[i].thd;
		  joint_des_state[i].thdd 	= currState[i].thdd;
		  joint_des_state[i].uff 	= 0.f;
//		  printf("%f, %f \n", joint_des_state[i].th, joint_state[i].th);
	}
//	  printf("\n");
//	  printf("\n______________________\n");
	  // uff is just ramped to the desired value
//	  for (i=1; i<=N_DOFS; ++i)
//	  {
//	    joint_des_state[i].uff  += joint_increment[i].uff;
//	  }

	  double error = 0.f;
	  FOR(i,N_DOFS)
	  {
		  error += ABS(goalState[i].th  - joint_des_state[i].th );
	  }


	  if(error < tolerance)
	  {
		  reachedGoal ++ ;
	  }
	  else
	  {
		  reachedGoal = 0;
	  }
	  reachedInitState = reachedGoal > 100;
	  counter ++;
	  //~ printf("Error : %f\n", error);
	  return reachedInitState;

}

int gotoPosStep(double joints[N_DOFS+1], double maxStep)
{
	static int i;
	static double jointStep[N_DOFS+1];

	if (inverseDynControl == -1)
	{
		printf("Please Initialize the motor utilities!\n");
		freeze();
	}

//	    printf("Hello\n");

	if (firstGotoPos == 1)
	{
		for (i = 1; i <= N_DOFS; i ++)
		{
			joint_des_state[i].th = joint_state[i].th;
			joint_des_state[i].thd = 0.0;
		}
		firstGotoPos = 0;
	}

	
	//	printf("%f\n", joints[1]);
	FOR(i,N_DOFS)
	{

	//		jointStep[i] = joints[i] - joint_des_state[i].th;
		jointStep[i] = joints[i] - joint_des_state[i].th;
		if (inverseDynControl)
		{

	//			joint_des_state[i].th = joints[i];
			joint_des_state[i].th = joint_state[i].th;
			joint_des_state[i].thd = 0.0;
			joint_des_state[i].thdd = kPos * (joints[i]  - joint_state[i].th) + kVel * (- joint_state[i].thd);
			if(fabs(joint_des_state[i].thdd) > maxAcceleration)
			{
				joint_des_state[i].thdd = maxAcceleration * SIGN(joint_des_state[i].thdd);
			}
			//~ printf("%f, ", jointStep[i]);
		}
		else
		{
			if( ABS(jointStep[i]) > maxStep)
			{
				jointStep[i] = SIGN(jointStep[i]) * maxStep;
			}
			joint_des_state[i].th  = joint_des_state[i].th + jointStep[i];
			joint_des_state[i].thd = 0.f;
			joint_des_state[i].thdd = 0.0;
			if(ABS(joint_des_state[i].th - joint_state[i].th) > maxStep)
			{
				joint_des_state[i].th = joint_state[i].th + maxStep * SIGN(joint_des_state[i].th - joint_state[i].th);
			}
		}

	}
	
	set_toJointLimits(joint_des_state, 0.1);

	double error = 0.f;
	FOR(i,N_DOFS)
	{
		error += ABS(jointStep[i]);
	}


	if(error < tolerance)
	{
		reachedGoal ++ ;
	}
	else
	{
		reachedGoal = 0;
	}
	return reachedGoal > 100 ;
}

int gotoPos(double joints[N_DOFS+1])
{
	double maxStep = 0.15;
	return gotoPosStep(joints,maxStep);
}

int doMotion(double joints[STEPLIMIT][N_DOFS+1], int step, int maxStep)  {

	if (inverseDynControl == -1) {
		printf("Please Initialize the motor utilities!\n");
		freeze();
	}

	static int i;
	int j = 0;
	static double integratePos[N_DOFS + 1];


	int  startTrajectory = step > 1;
	//~ printf("StartTrajectory: %d %d\n",startTrajectory, *step);

	if (step == 1) 	{
		//startTrajectory = (gotoPosMinJerk(joints[1]) );
		FOR(i,N_DOFS) {
			integratePos[i] = joints[1][i];
	    }
	}

	double next_velocity = 0;
	double velocity = 0;
	double old_velocity = 0;

	if(step > 1 && step <= maxStep) 	{

		FOR(i,N_DOFS) {


			double jPos_p1 = 0;
			double jPos_p2 = 0;

			if (step <= maxStep -1)
				jPos_p1 = joints[step + 1][i];
			else
				jPos_p1 = joints[step ][i];

			if (step <= maxStep - 2)
				jPos_p2 = joints[step + 2][i];
			else
				jPos_p2 = jPos_p1;

			next_velocity = (jPos_p2 - jPos_p1) * task_servo_rate;
			velocity = (jPos_p1 - joints[step][i]) * task_servo_rate;
			old_velocity = (joints[step][i] - joints[step - 1][i]) * task_servo_rate;

			if (inverseDynControl) {

				joint_des_state[i].th = joint_state[i].th;
				joint_des_state[i].thd = joint_state[i].thd;

//				joint_des_state[i].th = joints[step][i];
//				joint_des_state[i].thd = old_velocity;

//				joint_des_state[i].thdd = (velocity - old_velocity) * task_servo_rate;

//				joint_des_state[i].thdd = kPos * (joints[step][i]  - joint_state[i].th) +
//						kVel * (old_velocity - joint_state[i].thd) + (velocity - old_velocity) * task_servo_rate;

				joint_des_state[i].thdd = kPos * (joints[step][i]  - joint_state[i].th) +
						kVel * (old_velocity - joint_state[i].thd) + (next_velocity - velocity) * task_servo_rate;

			}
			else {

				double desiredVel = (joints[step][i] - joints[step - 1][i]) * task_servo_rate / t_ratio;

				if (fabs(desiredVel) > maxJointVel[i]) {
					desiredVel = maxJointVel[i] * SIGN(desiredVel);
					if (step > 1 && fabs(joints[step][i] - joints[step - 1][i]) > 0.2) {

						printf("Step %d is too large - FREEZE: %f %f!!!\n", step, joints[step][i], joints[step - 1][i]);
						for (j = 0; j < 10; j ++)
						{
							printf("%d: ", j);
							FOR(i,N_DOFS) {
								printf("%f", joints[j][i]);
								joint_des_state[j].th = joint_state[j].th;
							}
							printf("\n");
						}
						freeze();
						break;
					}
				}

				integratePos[i] += desiredVel / (task_servo_rate / t_ratio);
//				printf("%d: %f %f %f\n", i, desiredVel, integratePos[i], joints[step][i]);

				//printf("%f %f, ", joints[step][i] - joint_state[i].th, velocity);
				joint_des_state[i].th  = integratePos[i];
				joint_des_state[i].thd = desiredVel;
				joint_des_state[i].thdd = 0; //(velocity - old_velocity) / task_servo_rate;
			}
		}

	}
	stepInTrajectory = step;

	check_range(joint_des_state);
	set_toJointLimits(joint_des_state, 0.1);

	return 1;

}

/* step 0 ->
 * total_duration in idx of joints vector
 * */
int doControl( double joints[STEPLIMIT][N_DOFS+1],  int step, int ctl_duration, int total_duration) {

	int i,j;

	if (inverseDynControl == -1) {
		printf("Please Initialize the motor utilities!\n");
		freeze();
	}

	static Matrix K = 0;
	static Vector k = 0;
	static Vector u = 0;
	static Vector q = 0;

	if  ( K == 0 ) {
		K = my_matrix(1,N_DOFS,1,2*N_DOFS);
		k = my_vector(1,N_DOFS);
		u = my_vector(1,N_DOFS);
		q = my_vector(1,2*N_DOFS);
	}


	int new_idx = step / ctl_duration;

	int ok = 0;

	static int old_idx = -1;
	if ( old_idx != new_idx && new_idx*(2*N_DOFS+1)<=total_duration ) {

		static int K_idx = 0;

		if ( old_idx > new_idx )
			K_idx = 0;

		old_idx = new_idx;

		for ( i = 1; i <= N_DOFS; ++i ) {
				for ( j = 1; j <= 2*N_DOFS; ++j ) {
					K[i][j] = joints[j+K_idx][i];
				}
				k[i] = joints[2*N_DOFS+1+K_idx][i];
		}

		K_idx += 2*N_DOFS+1 ;

		//TODO not here
		for ( i = 1; i <= N_DOFS; ++i ) {
			q[i]                  = joint_state[i].th;
			q[i+N_DOFS] = joint_state[i].thd;
		}

		ok = mat_vec_mult(K,q,u);
		ok &= vec_add(u,k,u);

	}

//TODO option for constant action
//	for ( i = 1; i <= N_DOFS; ++i ) {
//		q[i]                  = joint_state[i].th;
//		q[i+N_DOFS] = joint_state[i].thd;
//	}

	//the actions should be hold still for ctl_duration
	//int ok = mat_vec_mult(K,q,u);
	//ok &= vec_add(u,k,u);

	//TODO check ok....


	for ( i = 1; i <= N_DOFS; ++i ) {

		joint_des_state[i].th = joint_state[i].th;  //Resetting the internal PD
		joint_des_state[i].thd = joint_state[i].thd;

		if (ok && inverseDynControl)
			joint_des_state[i].thdd = u[i];
		else if (ok)
			joint_des_state[i].uff = u[i];

	}

	check_range(joint_des_state);
	set_toJointLimits(joint_des_state, 0.1);

	return ok;

}


/*******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out]      state : the current state
 \param[in]           goal : the desired state
 \param[in]            tau : the desired movement duration until the goal is

 ******************************************************************************/
int ias_calculate_min_jerk_next_step (SL_DJstate *state, SL_DJstate *goal,double tau)
{
	double t1,t2,t3,t4,t5;
	double tau1,tau2,tau3,tau4,tau5;
	int    i,j;
	double delta_t;

	delta_t = 1./((double)task_servo_rate);

	if (delta_t > tau || delta_t <= 0) {
		return FALSE;
	}

	t1 = delta_t;
	t2 = t1 * delta_t;
	t3 = t2 * delta_t;
	t4 = t3 * delta_t;
	t5 = t4 * delta_t;

	tau1 = tau;
	tau2 = tau1 * tau;
	tau3 = tau2 * tau;
	tau4 = tau3 * tau;
	tau5 = tau4 * tau;

	for (j=1; j<=n_dofs; ++j) //TODO
	{

		/* calculate the constants */
		const double dist   = goal[j].th - state[j].th;
		const double p1     = goal[j].th;
		const double p0     = state[j].th;
		const double a1t2   = goal[j].thdd;
		const double a0t2   = state[j].thdd;
		const double v1t1   = goal[j].thd;
		const double v0t1   = state[j].thd;

		const double c1 = 6.*dist/tau5 + (a1t2 - a0t2)/(2.*tau3) -
		3.*(v0t1 + v1t1)/tau4;
		const double c2 = -15.*dist/tau4 + (3.*a0t2 - 2.*a1t2)/(2.*tau2) +
		(8.*v0t1 + 7.*v1t1)/tau3;
		const double c3 = 10.*dist/tau3+ (a1t2 - 3.*a0t2)/(2.*tau) -
		(6.*v0t1 + 4.*v1t1)/tau2;
		const double c4 = state[j].thdd/2.;
		const double c5 = state[j].thd;
		const double c6 = state[j].th;

		state[j].th   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1 + c6;
		state[j].thd  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
		state[j].thdd = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;

	}

	return TRUE;
}


