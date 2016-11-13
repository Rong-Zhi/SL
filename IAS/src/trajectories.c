#include "trajectories.h"

/*****************************************************************************
 *****************************************************************************
 Function Name : spline5th()
 Date          : Jan. 08

 Remarks: 

         computes 5th order spline. Acceleration and velocity for time 0 and t0 are zero.
	 t0 is a global variable.
 *****************************************************************************
 Parameters: 
	 joint saves the computet joint values, velocities and accelerations
         pa start point of the trajectory
	 pb stop point of the trajectory
	 t time step for the point in the trajectory

 ****************************************************************************/

void spline5th(SL_DJstate *joint, SL_DJstate *start, SL_DJstate *stop, double t, double T){
    static double a[N_DOFS+1],b[N_DOFS+1],c[N_DOFS+1],d[N_DOFS+1],f[N_DOFS+1],g[N_DOFS+1]; 
    int i; 
       
    for(i=1; i<=N_DOFS; i++){
         a[i]=start[i].th; 
         b[i]=start[i].thd;
	 c[i]=0.5*start[i].thdd; 
         d[i]=(10*(stop[i].th-start[i].th)-6*start[i].thd*T - 1.5*start[i].thdd*sqr(T))/(T*T*T); 
         f[i]=(-15*(stop[i].th-start[i].th)+8*start[i].thd*T + 1.5*start[i].thdd*sqr(T))/(T*T*T*T); 
         g[i]=(6*(stop[i].th-start[i].th)-3*start[i].thd*T - 0.5*start[i].thdd*sqr(T))/(T*T*T*T*T);
       }

      
    for(i=1; i<=N_DOFS; i++){
      
      joint[i].th   = a[i] + b[i]*t + c[i]*t*t + d[i]*t*t*t + f[i]*t*t*t*t + g[i]*t*t*t*t*t; 
      joint[i].thd  =        b[i]   + 2*c[i]*t + 3*d[i]*t*t + 4*f[i]*t*t*t + 5*t*t*t*t*g[i]; 
      joint[i].thdd =                 2*c[i]   + 6*d[i]*t   + 12*f[i]*t*t  + 20*g[i]*t*t*t;  

    }

}

void spline5thvia(SL_DJstate *joint, SL_DJstate *start, SL_DJstate *stop, SL_DJstate *viapoint, SL_endeff *endeff, SL_Cstate predVHP_I, double t, double T1, double T2, int update, int torque){
	if(torque) s5via_torque(joint, start, stop, viapoint, endeff, predVHP_I, t, T1, T2, update);
	else	   s5via(joint, start, stop, viapoint, t, T1, T2, update);
	int i;
	for(i=1; i<=N_DOFS; i++){
		joint_des_state[i].th = joint_state[i].th;
		joint_des_state[i].thd = joint_state[i].thd;
	}
}

/*****************************************************************************
 *****************************************************************************
 Function Name : trajectory2()
 Date          : Jan. 08

 Remarks: 

         computes trajectory with two 5th order splines. User defines start, stop and via point and the velocities for the via point. Acceleration and velocity for time 0 and t2 are zero.
	 t1 and t2 are a global variables.
 *****************************************************************************
 Parameters: 
	 joint saves the computet joint values, velocities and accelerations
         pa start point of the trajectory
	 pb stop point of the trajectory
	 pv via point
	 vv velocities at the viapoint
	 t time step for the point in the trajectory

 ****************************************************************************/

void s5via(SL_DJstate *joint, SL_DJstate *start, SL_DJstate *stop, SL_DJstate *viapoint, double t, double T1, double T2, int update){
    static double a[N_DOFS+1], b[N_DOFS+1], c[N_DOFS+1], f[N_DOFS+1],g[N_DOFS+1],h[N_DOFS+1],u[N_DOFS+1],
                  v[N_DOFS+1],x[N_DOFS+1],r[N_DOFS+1],q[N_DOFS+1],w[N_DOFS+1], z; 

    static  Vector xm[N_DOFS+1], bm[N_DOFS+1];
    static  Matrix M[N_DOFS+1]; 

    static int firsttime=1; 
 
    int i,j,k, crap; 
    double tmp_t = 0, time_step = 1./500.;

    if(firsttime){
	firsttime=0;

	for(i=1;i<=N_DOFS; i++){
	  M[i]  = my_matrix(1,12,1,12); 
          bm[i] = my_vector(1,12); 
          xm[i] = my_vector(1,12); 
        } 
   }

  tmp_t = t;

  if(update)	
  {
  	t -=time_step;

       for(i=1; i<=N_DOFS; i++){

	  z=0;

          bm[i][1]=start[i].th; bm[i][2]=viapoint[i].th; bm[i][3]=start[i].thd; bm[i][4]=viapoint[i].thd; bm[i][5]=start[i].thdd; bm[i][6]=z;  bm[i][7]=viapoint[i].th; bm[i][8]=stop[i].th; bm[i][9]=viapoint[i].thd; bm[i][10]=0; bm[i][11]=z; bm[i][12]=0;

          M[i][1][1]=1; M[i][1][2]=t;  M[i][1][3]=sqr(t); M[i][1][4]=cube(t); M[i][1][5]=sqr(sqr(t)); M[i][1][6]=sqr(t)*cube(t); M[i][1][7]=0; M[i][1][8]=0;  M[i][1][9]=0; M[i][1][10]=0; M[i][1][11]=0; M[i][1][12]=0; 
          M[i][2][1]=1; M[i][2][2]=T1; M[i][2][3]=sqr(T1); M[i][2][4]=cube(T1);  M[i][2][5]=sqr(sqr(T1)); M[i][2][6]=sqr(T1)*cube(T1);  M[i][2][7]=0; M[i][2][8]=0;  M[i][2][9]=0;  M[i][2][10]=0;   M[i][2][11]=0; M[i][2][12]=0; 
	  M[i][3][1]=0; M[i][3][2]=1; M[i][3][3]=2*t; M[i][3][4]=3*sqr(t); M[i][3][5]=4*cube(t); M[i][3][6]=5*sqr(sqr(t)); M[i][3][7]=0; M[i][3][8]=0;  M[i][3][9]=0; M[i][3][10]=0; M[i][3][11]=0; M[i][3][12]=0; 
	  M[i][4][1]=0; M[i][4][2]=1; M[i][4][3]=2*T1; M[i][4][4]=3*sqr(T1); M[i][4][5]=4*cube(T1); M[i][4][6]=5*sqr(sqr(T1)); M[i][4][7]=0; M[i][4][8]=0;  M[i][4][9]=0; M[i][4][10]=0;         M[i][4][11]=0; M[i][4][12]=0; 
	  M[i][5][1]=0; M[i][5][2]=0; M[i][5][3]=2; M[i][5][4]=6*t; M[i][5][5]=12*sqr(t); M[i][5][6]=20*cube(t);  M[i][5][7]=0; M[i][5][8]=0; M[i][5][9]=0; M[i][5][10]=0;  M[i][5][11]=0;  M[i][5][12]=0;  
	  M[i][6][1]=0; M[i][6][2]=0; M[i][6][3]=2; M[i][6][4]=6*T1; M[i][6][5]=12*sqr(T1); M[i][6][6]=20*cube(T1); M[i][6][7]=0; M[i][6][8]=0; M[i][6][9]=0; M[i][6][10]=0; M[i][6][11]=0; M[i][6][12]=0; 
	  M[i][7][1]=0; M[i][7][2]=0;  M[i][7][3]=0; M[i][7][4]=0; M[i][7][5]=0; M[i][7][6]=0; M[i][7][7]=1; M[i][7][8]=T1; M[i][7][9]=sqr(T1); M[i][7][10]=cube(T1); M[i][7][11]=sqr(sqr(T1)); M[i][7][12]=sqr(T1)*cube(T1); 
	  M[i][8][1]=0; M[i][8][2]=0; M[i][8][3]=0; M[i][8][4]=0; M[i][8][5]=0; M[i][8][6]=0; M[i][8][7]=1; M[i][8][8]=T2; M[i][8][9]=sqr(T2); M[i][8][10]=cube(T2); M[i][8][11]=sqr(sqr(T2)); M[i][8][12]=sqr(T2)*cube(T2); 
	  M[i][9][1]=0; M[i][9][2]=0; M[i][9][3]=0; M[i][9][4]=0; M[i][9][5]=0; M[i][9][6]=0; M[i][9][7]=0; M[i][9][8]=1;  M[i][9][9]=2*T1; M[i][9][10]=3*sqr(T1); M[i][9][11]=4*cube(T1);   M[i][9][12]=5*sqr(sqr(T1)); 
      	  M[i][10][1]=0; M[i][10][2]=0;  M[i][10][3]=0; M[i][10][4]=0; M[i][10][5]=0; M[i][10][6]=0; M[i][1][7]=0; M[i][10][8]=1; M[i][10][9]=2*T2; M[i][10][10]=3*sqr(T2); M[i][10][11]=4*cube(T2); M[i][10][12]=5*sqr(sqr(T2)); 
	  M[i][11][1]=0; M[i][11][2]=0; M[i][11][3]=0; M[i][11][4]=0; M[i][11][5]=0; M[i][11][6]=0; M[i][11][7]=0; M[i][11][8]=0;  M[i][11][9]=2; M[i][11][10]=6*T1; M[i][11][11]=12*sqr(T1);   M[i][11][12]=20*cube(T1); 
	  M[i][12][1]=0; M[i][12][2]=0;  M[i][12][3]=0; M[i][12][4]=0; M[i][12][5]=0; M[i][12][6]=0; M[i][12][7]=0; M[i][12][8]=0;  M[i][12][9]=2;  M[i][12][10]=6*T2; M[i][12][11]=12*sqr(T2);   M[i][12][12]=20*cube(T2);

          if(!my_inv_ludcmp_solve(M[i], bm[i],12, xm[i])){
		print_mat("M",M[i]);
		int asdf = scanf("%d",&crap);
	  }

          a[i]=xm[i][1]; b[i]=xm[i][2]; c[i]=xm[i][3]; f[i]=xm[i][4]; g[i]=xm[i][5]; h[i]=xm[i][6]; u[i]=xm[i][7]; v[i]=xm[i][8];  x[i]=xm[i][9];  r[i]=xm[i][10]; q[i]=xm[i][11];  w[i]=xm[i][12];

        }
    }

    t = tmp_t;
    
    for(i=1; i<=N_DOFS; i++){
      if(t<=T1){
         joint[i].th   = a[i] + b[i]*t + c[i]*sqr(t) + f[i]*cube(t)  + g[i]*pow(t,4) + h[i]*pow(t,5);   
         joint[i].thd  =         b[i]   + 2*c[i]*t    + 3*f[i]*sqr(t) + 4*g[i]*cube(t)  + 5*h[i]*pow(t,4);  
         joint[i].thdd =                  2*c[i]      + 6*f[i]*t      + 12*g[i]*sqr(t)  + 20*h[i]*cube(t);   
      }
      else {
         joint[i].th   = u[i] + v[i]*t + x[i]*sqr(t) + r[i]*cube(t)  + q[i]*pow(t,4) + w[i]*pow(t,5);   
         joint[i].thd  =         v[i]   + 2*x[i]*t    + 3*r[i]*sqr(t) + 4*q[i]*cube(t)  + 5*w[i]*pow(t,4); 
         joint[i].thdd =                  2*x[i]      + 6*r[i]*t      + 12*q[i]*sqr(t)  + 20*w[i]*cube(t); 
      }
 
    }

}


// splines with torque limits (new version), compute acceleration at hitting point such that torque is minimized...
static void s5via_torque(SL_DJstate *joint, SL_DJstate *start, SL_DJstate *stop, SL_DJstate *viapoint, SL_endeff *endeff, SL_Cstate predVHP_I, double t, double T1, double T2, int update){
    //variables for trajectory planning
    static double a[N_DOFS+1], b[N_DOFS+1], c[N_DOFS+1], f[N_DOFS+1],g[N_DOFS+1],h[N_DOFS+1],u[N_DOFS+1],
                  v[N_DOFS+1],x[N_DOFS+1],r[N_DOFS+1],q[N_DOFS+1],p[N_DOFS+1], z; 
    
    static  Vector xm1, xm2, bm1, bm2;
    static  Matrix M1, M2;

    //variables for torque estimation
    static SL_endeff d_endeff[N_ENDEFFS+1];  
    static SL_DJstate d_a[N_DOFS+1];
    static double w[N_DOFS+1], m[N_DOFS+1], lc[N_DOFS+1], l[N_DOFS+1];
    static double sum=0., sum2=0., sum3=0., t1, t2, d, e;
    static double at[N_DOFS+1], bt[N_DOFS+1], ct[N_DOFS+1], ft[N_DOFS+1],gt[N_DOFS+1],ht[N_DOFS+1],ut[N_DOFS+1],
                  vt[N_DOFS+1], xt[N_DOFS+1], rt[N_DOFS+1], qt[N_DOFS+1],pt[N_DOFS+1]; 
    double ll1, ll2, ll3;


    // rest 
    static int firsttime=1; 
 
    int i,j,k, torque=0; 
    static double tmp_t = 0, time_step = 1./500.;
    double t_tmp =0;

    ll1= 0.55;  ll2 = 0.3;  ll3 = 0.08;

    if(firsttime){
	firsttime=0;

        M1 = my_matrix(1,6,1,6); 
	M2 = my_matrix(1,6,1,6);
        bm1 = my_vector(1,6); 
	bm2 = my_vector(1,6);
        xm1 = my_vector(1,6);
	xm2 = my_vector(1,6); 

   }

   tmp_t = t;

   if(update){
	//torque
	m[1] = 0.; m[2] = 2.4; m[3] = 5.0; m[4] = 1.46;
	m[5] = 1.44; m[6] = 0.93; m[7] = 0.12+0.3;

	l[1] = l[2] = l[3] = 0.;
	l[4] = ll1;
	l[5] = l[6] = l[7] = ll1+ll2;

	for(i=1; i<=3; i++){
		lc[i] = ll1/2.;
		lc[i+4] = ll3/2.;
	}
	lc[4] = ll2/2.;

	for(i=1; i<=N_DOFS; i++){
		sum = 0.;
		for(j=i+1; j<= N_DOFS; j++)
			sum+=m[j]*sqr(lc[j]+l[j]);
		w[i] = m[i]*sqr(lc[i])+sum;
	}

	sum = 0.; sum2 = 0.; sum3 = 0.;

	// trajectory planning
	t -=time_step;
	if(t<0) t +=time_step;

        for(i=1; i<=N_DOFS; i++){
	  z = -(((2*sqrt(6)-3)*t+(3-2*sqrt(6))*T1)*start[i].thd+((4*sqrt(6)-1)*t+(1-4*sqrt(6))*T1)*viapoint[i].thd+(4-pow(6,3./2.))*start[i].th+ (pow(6,3./2.)-4)*viapoint[i].th)/((sqrt(6)+1)*sqr(t)+(-2*sqrt(6)-2)*T1*t+(sqrt(6)+1)*sqr(T1));
	  z=0;

//	  stop[i].th = viapoint[i].th + 0.45*viapoint[i].thd*(T2-T1);

//	  if(stop[i].th<joint_range[i][MIN_THETA]){
//		stop[i].th = viapoint[i].th  + joint_range[i][MIN_THETA] - stop[i].th;
//	  } else if(stop[i].th>joint_range[i][MAX_THETA]){
//		stop[i].th = viapoint[i].th +joint_range[i][MAX_THETA] - stop[i].th;
//	  }

          bm1[1]=start[i].th;   bm1[2]=viapoint[i].th; bm1[3]=start[i].thd;    bm1[4]=viapoint[i].thd; bm1[5]=start[i].thdd; bm1[6]=z;  
	  bm2[1]=viapoint[i].th;  bm2[2]=stop[i].th;     bm2[3]=viapoint[i].thd;   bm2[4]=0.;              bm2[5]=z;               bm2[6]=0;

          M1[1][1]=1; M1[1][2]=t;  M1[1][3]=sqr(t);  M1[1][4]=cube(t);   M1[1][5]=pow(t,4);     M1[1][6]=pow(t,5); 
          M1[2][1]=1; M1[2][2]=T1; M1[2][3]=sqr(T1); M1[2][4]=cube(T1);  M1[2][5]=sqr(sqr(T1)); M1[2][6]=pow(T1,5);
	  M1[3][1]=0; M1[3][2]=1;  M1[3][3]=2*t;     M1[3][4]=3*sqr(t);  M1[3][5]=4*cube(t);    M1[3][6]=5*pow(t,4);  
	  M1[4][1]=0; M1[4][2]=1;  M1[4][3]=2*T1;    M1[4][4]=3*sqr(T1); M1[4][5]=4*cube(T1);   M1[4][6]=5*pow(T1,4); 
	  M1[5][1]=0; M1[5][2]=0;  M1[5][3]=2;       M1[5][4]=6*t;       M1[5][5]=12*sqr(t);    M1[5][6]=20*cube(t); 
	  M1[6][1]=0; M1[6][2]=0;  M1[6][3]=2;       M1[6][4]=6*T1;      M1[6][5]=12*sqr(T1);   M1[6][6]=20*cube(T1); 

	  M2[1][1]=1; M2[1][2]=T1; M2[1][3]=sqr(T1); M2[1][4]=cube(T1);  M2[1][5]=pow(T1,4);    M2[1][6]=pow(T1,5); 
	  M2[2][1]=1; M2[2][2]=T2; M2[2][3]=sqr(T2); M2[2][4]=cube(T2);  M2[2][5]=pow(T2,4);    M2[2][6]=pow(T2,5); 
	  M2[3][1]=0; M2[3][2]=1;  M2[3][3]=2*T1;    M2[3][4]=3*sqr(T1); M2[3][5]=4*cube(T1);   M2[3][6]=5*pow(T1,4); 
      	  M2[4][1]=0; M2[4][2]=1;  M2[4][3]=2*T2;    M2[4][4]=3*sqr(T2); M2[4][5]=4*cube(T2);   M2[4][6]=5*pow(T2,4); 
	  M2[5][1]=0; M2[5][2]=0;  M2[5][3]=2;       M2[5][4]=6*T1;      M2[5][5]=12*sqr(T1);   M2[5][6]=20*cube(T1); 
	  M2[6][1]=0; M2[6][2]=0;  M2[6][3]=2;       M2[6][4]=6*T2;      M2[6][5]=12*sqr(T2);   M2[6][6]=20*cube(T2);

          if(!my_inv_ludcmp_solve(M1, bm1,6, xm1))
	  {
		//printf("Error in trajectory3\n");
		//freeze();
		return;
	  }

	  if(!my_inv_ludcmp_solve(M2, bm2, 6, xm2)){
		//printf("Error in trajector3\n");
		//freeze();
		return;
	  }

//	 print_mat("M1",M1);
 //        print_vec("x",xm1);
//	 print_vec("b",bm1);

          at[i]=xm1[1]; bt[i]=xm1[2]; ct[i]=xm1[3]; ft[i]=xm1[4]; gt[i]=xm1[5]; ht[i]=xm1[6]; 
	  ut[i]=xm2[1]; vt[i]=xm2[2]; xt[i]=xm2[3]; rt[i]=xm2[4]; qt[i]=xm2[5]; pt[i]=xm2[6];

	  //torque
	  sum+= w[i]*ft[i];
	  sum2+= w[i]*gt[i];
	  sum3+= w[i]*ht[i];

        }
    }

    t = tmp_t;

    torque = 0;
    if(t<T1){
	d = (2/5.)*(sum2/sum3);
	e = sum/(10*sum3);

	t1 = -d/2. + sqrt(sqr(d)/4. -e);
	t2 = -d/2. - sqrt(sqr(d)/4. -e);

	for(i=1; i<=N_CART; i++){
		d_endeff[1].mcm[i] = endeff[1].mcm[i];
		d_endeff[1].x[i] = predVHP_I.x[i];
		d_endeff[1].a[i] = endeff[1].a[i];
	}

	  //check t1
	  for(i=1; i<=N_DOFS; i++){
		d_a[i].th   =  at[i] + bt[i]*t1 + ct[i]*sqr(t1) + ft[i]*cube(t1)  + gt[i]*pow(t1,4) + ht[i]*pow(t1,5);
		d_a[i].thd  =         bt[i]   + 2*ct[i]*t1    + 3*ft[i]*sqr(t1) + 4*gt[i]*cube(t1)  + 5*ht[i]*pow(t1,4);  
		d_a[i].thdd =                  2*ct[i]      + 6*ft[i]*t1      + 12*gt[i]*sqr(t1)  + 20*ht[i]*cube(t1);  
	  }


	  SL_InvDyn(NULL, d_a, d_endeff, &base_state,&base_orient);
	  if(t1<0.35 && t1>0. && (d_a[1].uff>=72 || d_a[2].uff>=122 || d_a[3].uff>=35 || d_a[4].uff>=28 || d_a[5].uff>=2.8 || d_a[6].uff>=3.8 || d_a[7].uff>=1.9)){
		torque = 1;
		//printf("TORQUE\n");
	}

	  // check t2
	  for(i=1; i<=N_DOFS; i++){
		d_a[i].th   =  at[i] + bt[i]*t2 + ct[i]*sqr(t2) + ft[i]*cube(t2)  + gt[i]*pow(t2,4) + ht[i]*pow(t2,5);
		d_a[i].thd  =         bt[i]   + 2*ct[i]*t2    + 3*ft[i]*sqr(t2) + 4*gt[i]*cube(t2)  + 5*ht[i]*pow(t2,4);  
		d_a[i].thdd =                  2*ct[i]      + 6*ft[i]*t2      + 12*gt[i]*sqr(t2)  + 20*ht[i]*cube(t2);  
	  }

	  SL_InvDyn(NULL, d_a, d_endeff, &base_state,&base_orient);
	  if(t2<0.35 && t2>0. && (d_a[1].uff>=72 || d_a[2].uff>=122 || d_a[3].uff>=35 || d_a[4].uff>=28 || d_a[5].uff>=2.8 || d_a[6].uff>=3.8 || d_a[7].uff>=1.9))
		torque = 1;

	  if(!torque || t<=0.02){
		for(i=1; i<=N_DOFS; i++){
			a[i] = at[i]; b[i] = bt[i]; c[i] = ct[i]; 
			f[i] = ft[i]; g[i] = gt[i]; h[i] = ht[i];

			u[i] = ut[i]; v[i] = vt[i]; x[i] = xt[i]; 
			r[i] = rt[i]; q[i] = qt[i]; p[i] = pt[i];
		}
	  }
    }


    for(i=1; i<=N_DOFS; i++){
      if(t<=T1){
         joint[i].th   = a[i] + b[i]*t + c[i]*sqr(t) + f[i]*cube(t)  + g[i]*pow(t,4) + h[i]*pow(t,5);   
         joint[i].thd  =        b[i]   + 2*c[i]*t    + 3*f[i]*sqr(t) + 4*g[i]*cube(t)  + 5*h[i]*pow(t,4);  
         joint[i].thdd =                 2*c[i]      + 6*f[i]*t      + 12*g[i]*sqr(t)  + 20*h[i]*cube(t);   
      }
      else {
         joint[i].th   = u[i] + v[i]*t + x[i]*sqr(t) + r[i]*cube(t)  + q[i]*pow(t,4) + p[i]*pow(t,5);   
         joint[i].thd  =         v[i]   + 2*x[i]*t    + 3*r[i]*sqr(t) + 4*q[i]*cube(t)  + 5*p[i]*pow(t,4); 
         joint[i].thdd =                  2*x[i]      + 6*r[i]*t      + 12*q[i]*sqr(t)  + 20*p[i]*cube(t); 
      }
 
    }

}

