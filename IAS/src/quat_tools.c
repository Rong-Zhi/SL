#include "quat_tools.h"

void rotate_quat(SL_quat q, Vector v, Vector v_new)
{ 
  double a, b, c, d, t2, t3, t4, t5, t6, t7, t8, t9, t10;

  a = q.q[1]; b = q.q[2]; c = q.q[3]; d = q.q[4];
  t2 =   a*b;
  t3 =   a*c;
  t4 =   a*d;
  t5 =  -b*b;
  t6 =   b*c;
  t7 =   b*d;
  t8 =  -c*c;
  t9 =   c*d;
  t10 = -d*d;
  v_new[1] = 2*( (t8 + t10)*v[1] + (t6 -  t4)*v[2] + (t3 + t7)*v[3] ) + v[1];
  v_new[2] = 2*( (t4 +  t6)*v[1] + (t5 + t10)*v[2] + (t9 - t2)*v[3] ) + v[2];
  v_new[3] = 2*( (t7 - t3)*v[1] + (t2 + t9)*v[2] + (t5 + t8)*v[3] ) + v[3];
}


void rotate_negative_quat(SL_quat q, Vector v, Vector v_new)
{ 
  double a, b, c, d, t2, t3, t4, t5, t6, t7, t8, t9, t10;

  a = q.q[1]; b = - q.q[2]; c = - q.q[3]; d = - q.q[4];
  t2 =   a*b;
  t3 =   a*c;
  t4 =   a*d;
  t5 =  -b*b;
  t6 =   b*c;
  t7 =   b*d;
  t8 =  -c*c;
  t9 =   c*d;
  t10 = -d*d;
  v_new[1] = 2*( (t8 + t10)*v[1] + (t6 -  t4)*v[2] + (t3 + t7)*v[3] ) + v[1];
  v_new[2] = 2*( (t4 +  t6)*v[1] + (t5 + t10)*v[2] + (t9 - t2)*v[3] ) + v[2];
  v_new[3] = 2*( (t7 - t3)*v[1] + (t2 + t9)*v[2] + (t5 + t8)*v[3] ) + v[3];
}


// Compute quaternion to go from v to v_new
void rotate_quat_inv(SL_quat *q, Vector v, Vector v_new)
{
	static int firsttime = 1;
	double a,b,c,d,t2,t4,t5,t6,t7,t8,t9,t10,norm;
	double theta;

	double u[3];
	theta = acos((v[1]*v_new[1]+v[2]*v_new[2] + v[3]*v_new[3])/(sqrt(v[1]*v[1] + v[2]*v[2] + v[3]*v[3])*sqrt(v_new[1]*v_new[1] + v_new[2]*v_new[2] + v_new[3]*v_new[3])));
	theta /= 2.;

	//crossproduct
	u[0] = v[2]*v_new[3] - v[3]*v_new[2];
	u[1] = v[3]*v_new[1] - v[1]*v_new[3];
	u[2] = v[1]*v_new[2] - v[2]*v_new[1];	

	//normalize
	norm = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
	u[0] /= norm;
	u[1] /= norm;
	u[2] /= norm;

	//orientation
	(*q).q[1] = cos(theta);
	(*q).q[2] = u[0]*sin(theta);
	(*q).q[3] = u[1]*sin(theta);
	(*q).q[4] = u[2]*sin(theta);
}


void quat_vec_mult(SL_quat q, Vector in, Vector out)
{
	static Matrix Q;
	static int firsttime =1;

	double q0,q1,q2,q3;

	if(firsttime)
	{
		firsttime = 0;

		Q = my_matrix(1,3,1,3);
	}

	q0 = q.q[_Q0_];
	q1 = q.q[_Q1_];
	q2 = q.q[_Q2_];
	q3 = q.q[_Q3_];

	Q[1][1]=2*sqr(q0)-1+2*sqr(q1);	Q[1][2]=2*q1*q2-2*q0*q3;	Q[1][3]=2*q1*q3+2*q0*q2;
	Q[2][1]=2*q1*q2+2*q0*q3;	Q[2][2]=2*sqr(q0)-1+2*sqr(q2);	Q[2][3]=2*q2*q3-2*q0*q1;
	Q[3][1]=2*q1*q3-2*q0*q3;	Q[3][2]=2*q2*q3+2*q0*q1;	Q[3][3]=2*sqr(q0)-1+2*sqr(q3);

	mat_vec_mult(Q,in,out);
}


/*****************************************************************************
******************************************************************************
 Function Name    : mult_quat
 Date        : March 2008

 multiplies 2 quaternions

*****************************************************************************/

void mult_quat(SL_quat *q1, SL_quat *q2, SL_quat *q_new) {
 double a, b, c, d, e, f, g, h;

 a = (*q1).q[1]; b = (*q1).q[2]; c = (*q1).q[3]; d = (*q1).q[4];
 e = (*q2).q[1]; f = (*q2).q[2]; g = (*q2).q[3]; h = (*q2).q[4];
 (*q_new).q[1] = a*e - b*f - c*g - d*h;
 (*q_new).q[2] = b*e + a*f + c*h - d*g;
 (*q_new).q[3] = a*g - b*h + c*e + d*f;
 (*q_new).q[4] = a*h + b*g - c*f + d*e;
};
