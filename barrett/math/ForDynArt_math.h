/* sine and cosine precomputation */
sstate1th=Sin(state[1].th);
cstate1th=Cos(state[1].th);

sstate2th=Sin(state[2].th);
cstate2th=Cos(state[2].th);

sstate3th=Sin(state[3].th);
cstate3th=Cos(state[3].th);

sstate4th=Sin(state[4].th);
cstate4th=Cos(state[4].th);

sstate5th=Sin(state[5].th);
cstate5th=Cos(state[5].th);

sstate6th=Sin(state[6].th);
cstate6th=Cos(state[6].th);

sstate7th=Sin(state[7].th);
cstate7th=Cos(state[7].th);


/* rotation matrix sine and cosine precomputation */







rseff1a1=Sin(eff[1].a[1]);
rceff1a1=Cos(eff[1].a[1]);

rseff1a2=Sin(eff[1].a[2]);
rceff1a2=Cos(eff[1].a[2]);

rseff1a3=Sin(eff[1].a[3]);
rceff1a3=Cos(eff[1].a[3]);



_ForDynArtfunc1();

_ForDynArtfunc2();

_ForDynArtfunc3();

_ForDynArtfunc4();

_ForDynArtfunc5();

_ForDynArtfunc6();

_ForDynArtfunc7();

_ForDynArtfunc8();

_ForDynArtfunc9();

_ForDynArtfunc10();

_ForDynArtfunc11();

_ForDynArtfunc12();

_ForDynArtfunc13();

_ForDynArtfunc14();

_ForDynArtfunc15();

_ForDynArtfunc16();

/* acceleration vectors and joint accelerations */
state[1].thdd=u[1]/d[1];

a1[1]=c1[1];
a1[2]=c1[2];
a1[3]=state[1].thdd;
a1[4]=c1[4];
a1[5]=c1[5];

state[2].thdd=(a1[4]*h2[3] + a1[1]*h2[6] + u[2] - a1[5]*(h2[1]*S21[1][2] + h2[2]*S21[2][2]) - a1[2]*(h2[4]*S21[1][2] + h2[5]*S21[2][2]) - a1[3]*(h2[4]*S21[1][3] + h2[5]*S21[2][3]))/d[2];

a2[1]=c2[1] + a1[2]*S21[1][2] + a1[3]*S21[1][3];
a2[2]=c2[2] + a1[2]*S21[2][2] + a1[3]*S21[2][3];
a2[3]=state[2].thdd - a1[1];
a2[4]=c2[4] + a1[5]*S21[1][2];
a2[5]=c2[5] + a1[5]*S21[2][2];
a2[6]=-a1[4];

state[3].thdd=(-(a2[4]*h3[3]) - a2[1]*h3[6] + u[3] - a2[5]*(h3[1]*S32[1][2] + h3[2]*S32[2][2]) - a2[6]*(h3[1]*S32[1][3] + h3[2]*S32[2][3]) - a2[2]*(h3[4]*S32[1][2] - ZHR*h3[1]*S32[1][3] + h3[5]*S32[2][2] - ZHR*h3[2]*S32[2][3]) - a2[3]*(ZHR*h3[1]*S32[1][2] + h3[4]*S32[1][3] + ZHR*h3[2]*S32[2][2] + h3[5]*S32[2][3]))/d[3];

a3[1]=c3[1] + a2[2]*S32[1][2] + a2[3]*S32[1][3];
a3[2]=c3[2] + a2[2]*S32[2][2] + a2[3]*S32[2][3];
a3[3]=state[3].thdd + a2[1];
a3[4]=c3[4] + ZHR*a2[3]*S32[1][2] + a2[5]*S32[1][2] - ZHR*a2[2]*S32[1][3] + a2[6]*S32[1][3];
a3[5]=c3[5] + ZHR*a2[3]*S32[2][2] + a2[5]*S32[2][2] - ZHR*a2[2]*S32[2][3] + a2[6]*S32[2][3];
a3[6]=a2[4];

state[4].thdd=(a3[4]*h4[3] + u[4] - a3[5]*(h4[1]*S43[1][2] + h4[2]*S43[2][2]) - a3[2]*(-(ZEB*h4[3]) + h4[4]*S43[1][2] + h4[5]*S43[2][2]) - a3[6]*(h4[1]*S43[1][3] + h4[2]*S43[2][3]) - a3[3]*(YEB*h4[3] + h4[4]*S43[1][3] + h4[5]*S43[2][3]) - a3[1]*(-h4[6] + h4[1]*(-(ZEB*S43[1][2]) + YEB*S43[1][3]) + h4[2]*(-(ZEB*S43[2][2]) + YEB*S43[2][3])))/d[4];

a4[1]=c4[1] + a3[2]*S43[1][2] + a3[3]*S43[1][3];
a4[2]=c4[2] + a3[2]*S43[2][2] + a3[3]*S43[2][3];
a4[3]=state[4].thdd - a3[1];
a4[4]=c4[4] + a3[5]*S43[1][2] + a3[6]*S43[1][3] + a3[1]*(-(ZEB*S43[1][2]) + YEB*S43[1][3]);
a4[5]=c4[5] + a3[5]*S43[2][2] + a3[6]*S43[2][3] + a3[1]*(-(ZEB*S43[2][2]) + YEB*S43[2][3]);
a4[6]=-(ZEB*a3[2]) + YEB*a3[3] - a3[4];

state[5].thdd=(-(a4[4]*h5[3]) + u[5] - a4[5]*(h5[1]*S54[1][2] + h5[2]*S54[2][2]) - a4[6]*(h5[1]*S54[1][3] + h5[2]*S54[2][3]) - a4[1]*(h5[6] + YWR*h5[1]*S54[1][3] + YWR*h5[2]*S54[2][3]) - a4[2]*(h5[4]*S54[1][2] - ZWR*h5[1]*S54[1][3] + h5[5]*S54[2][2] - ZWR*h5[2]*S54[2][3]) - a4[3]*(-(YWR*h5[3]) + ZWR*h5[1]*S54[1][2] + h5[4]*S54[1][3] + ZWR*h5[2]*S54[2][2] + h5[5]*S54[2][3]))/d[5];

a5[1]=c5[1] + a4[2]*S54[1][2] + a4[3]*S54[1][3];
a5[2]=c5[2] + a4[2]*S54[2][2] + a4[3]*S54[2][3];
a5[3]=state[5].thdd + a4[1];
a5[4]=c5[4] + ZWR*a4[3]*S54[1][2] + a4[5]*S54[1][2] + YWR*a4[1]*S54[1][3] - ZWR*a4[2]*S54[1][3] + a4[6]*S54[1][3];
a5[5]=c5[5] + ZWR*a4[3]*S54[2][2] + a4[5]*S54[2][2] + YWR*a4[1]*S54[2][3] - ZWR*a4[2]*S54[2][3] + a4[6]*S54[2][3];
a5[6]=-(YWR*a4[3]) + a4[4];

state[6].thdd=(a5[4]*h6[3] + u[6] - a5[5]*(h6[1]*S65[1][2] + h6[2]*S65[2][2]) - a5[1]*(-h6[6] - ZWFE*h6[1]*S65[1][2] - ZWFE*h6[2]*S65[2][2]) - a5[2]*(-(ZWFE*h6[3]) + h6[4]*S65[1][2] + h6[5]*S65[2][2]) - a5[6]*(h6[1]*S65[1][3] + h6[2]*S65[2][3]) - a5[3]*(h6[4]*S65[1][3] + h6[5]*S65[2][3]))/d[6];

a6[1]=c6[1] + a5[2]*S65[1][2] + a5[3]*S65[1][3];
a6[2]=c6[2] + a5[2]*S65[2][2] + a5[3]*S65[2][3];
a6[3]=state[6].thdd - a5[1];
a6[4]=c6[4] - ZWFE*a5[1]*S65[1][2] + a5[5]*S65[1][2] + a5[6]*S65[1][3];
a6[5]=c6[5] - ZWFE*a5[1]*S65[2][2] + a5[5]*S65[2][2] + a5[6]*S65[2][3];
a6[6]=-(ZWFE*a5[2]) - a5[4];

state[7].thdd=(-(a6[4]*h7[3]) - a6[1]*h7[6] + u[7] - a6[5]*(h7[1]*S76[1][2] + h7[2]*S76[2][2]) - a6[2]*(h7[4]*S76[1][2] + h7[5]*S76[2][2]) - a6[6]*(h7[1]*S76[1][3] + h7[2]*S76[2][3]) - a6[3]*(h7[4]*S76[1][3] + h7[5]*S76[2][3]))/d[7];

a7[1]=c7[1] + a6[2]*S76[1][2] + a6[3]*S76[1][3];
a7[2]=c7[2] + a6[2]*S76[2][2] + a6[3]*S76[2][3];
a7[3]=state[7].thdd + a6[1];
a7[4]=c7[4] + a6[5]*S76[1][2] + a6[6]*S76[1][3];
a7[5]=c7[5] + a6[5]*S76[2][2] + a6[6]*S76[2][3];
a7[6]=a6[4];

a8[1]=a7[1]*S87[1][1] + a7[2]*S87[1][2] + a7[3]*S87[1][3];
a8[2]=a7[1]*S87[2][1] + a7[2]*S87[2][2] + a7[3]*S87[2][3];
a8[3]=a7[1]*S87[3][1] + a7[2]*S87[3][2] + a7[3]*S87[3][3];
a8[4]=a7[4]*S87[1][1] + a7[5]*S87[1][2] + a7[3]*(-(eff[1].x[2]*S87[1][1]) + eff[1].x[1]*S87[1][2]) + a7[6]*S87[1][3] + a7[2]*(eff[1].x[3]*S87[1][1] - eff[1].x[1]*S87[1][3]) + a7[1]*(-(eff[1].x[3]*S87[1][2]) + eff[1].x[2]*S87[1][3]);
a8[5]=a7[4]*S87[2][1] + a7[5]*S87[2][2] + a7[3]*(-(eff[1].x[2]*S87[2][1]) + eff[1].x[1]*S87[2][2]) + a7[6]*S87[2][3] + a7[2]*(eff[1].x[3]*S87[2][1] - eff[1].x[1]*S87[2][3]) + a7[1]*(-(eff[1].x[3]*S87[2][2]) + eff[1].x[2]*S87[2][3]);
a8[6]=a7[4]*S87[3][1] + a7[5]*S87[3][2] + a7[3]*(-(eff[1].x[2]*S87[3][1]) + eff[1].x[1]*S87[3][2]) + a7[6]*S87[3][3] + a7[2]*(eff[1].x[3]*S87[3][1] - eff[1].x[1]*S87[3][3]) + a7[1]*(-(eff[1].x[3]*S87[3][2]) + eff[1].x[2]*S87[3][3]);

