(* ::Package:: *)

(* $Id: RidgidBodyDynamics.m,v 1.0 1997/01/14 23:29:00 emily Exp $ *)

(* :Copyright: Copyright 1997-2011, Stefan Schaal *)

(*:Mathematica Version: 2.0 *)

(*:Name: ControlTheory`RigidBodyDynamics` *)
\[Florin]
(*:Title: Recursive Newton Euler Algorithms For Kinematic Chains *)

(*:Author: Stefan Schaal *)

(*:Keywords:
	inverse dynamics, spatial vector arithmitic, control theory, 
	ridgid body dynamics, forward dynamics, Jacobians
*)

(*:Requirements: Cross.m *)

(*:Warnings: None. *)

(*:Sources:
	Robot Dynamics Algorithms, by Roy Featherstone. 
	Kluwer Academic Publishers, 1987.
*)

(*:Summary:
This package derives the ridgid body dynamics equations for an open
kinematic chain and generates the equations in C format. An input
file with specific Mathematic syntax is required for specifications.
*)

BeginPackage["ControlTheory`RigidBodyDynamics`"]
		
(*  Recursive Newton Euler Inverse Dynamics  *)		
InvDynNE::usage = 
"InvDynNE[infile, outfile,gravity] derives the Newton Euler inverse dynamics 
equations from the specification in infile and dumps C-code output to outfile. 
The gravity vector in world coordinates is given (note that the gravity is 
supposed to be given WITH the appropriate sign!). The following rules apply:\n
-input files are in Mathematica notation and can use Mathematica symbolic
math\n
-joints in the input file are numbered by integer numbers. DO NOT use the
number 0 as it is used internally to refer to the base coordinate system.
The numbers provided will be used as indices for arrays in the C-Code.\n
-branches are permitted, but no loops.\n
-each joint must rotate or translate about one defined axis in its local coordinate system\n
-each local coordinate system has is origin at the joint\n
-the inertia tensor is in the center of mass coordinate system\n
-rotation angles for coordinate transformation are alpha (rotate about x-axis), 
beta (rotate about y-axis), gamma (rotate about z-axis) in this 
sequence, and in Euler angle notation\n
-do NEVER use underscores and dashes in variable names in the input file (Mathematica syntax)\n
-the rotation angles to get to the next local coordinate 
systems should be numerical if possibe (this creates more efficient code)\n";
	   

InertiaMatrix::usage = 
"InertiaMatrix[infile, outfile] derives the inertia matrix equations
from the specification in infile using composite inertias and dumps C-code
output to outfile. This program does not work for floating bases.\n
Check help on InvDyn for general rules how to greate
input files.\n";
	   

ForDynComp::usage = 
"ForDynComp[infile, outfile,gravity] derives the forward dynamics equations
from the specification in infile using composite inertias and dumps C-code
output to outfile. The gravity vector in world coordinates is given. The forward
dynamics code needs an implementation of a matrix inversion at the end, which must
be filled in manually (e.g., use Numerical Recipes LDLT inversion). The composite
inertia method is useful for systems with no floating base up to about 9 DOF. For
more DOFs or floating bases, use ForDynArt.\n
Check help on InvDyn for general rules how to greate
input files.\n";
	   

ForDynArt::usage = 
"ForDyn[infile, outfile,gravity] derives the forward dynamics equations from the 
specification in infile with the Ariticulated Body Algorithm and dumps C-code
output to outfile. The gravity vector in world coordinates is given. For floating
forward bases, the dynamics code needs an implementation of a matrix inversion at 
the end, which may need some manually modification(e.g., use Numerical Recipes 
LDLT inversion).\n 
Check help on InvDyn for general rules how to greate
input files.\n";
	   

InvDynArt::usage = 
"InvDyn[infile, outfile,gravity] derives the inverse dynamics equations from the 
specification in infile with the Ariticulated Body Algorithm and dumps C-code
output to outfile. The gravity vector in world coordinates is given. For floating
forward bases, the dynamics code needs an implementation of a matrix inversion at 
the end, which may need some manually modification(e.g., use Numerical Recipes 
LDLT inversion).\n 
Check help on InvDyn for general rules how to greate
input files.\n";
	   

LinkEndpointKinematics::usage = 
"LinkEndpointKinematics[infile, outfile] derives the Cartesian positions 
of each link from the specification in infile and dumps C-code
output to outfile. The link endpoints are useful for generating graphics,
or for checking collisions. Check help on InvDyn for general rules how to greate
input files.\n";
	   

GeometricJacobian::usage = 
"GeometricJacobian[infile,endeffs,outfile] derives the geometric Jacobian for
the endeffectors. endeffs contains the list of ID number for which the Jacobian
is derived.\n";
	   

LinkRotationMatrix::usage = 
"LinkRotationMatrix[infile,endeffs,outfile] derives the rotation matrix for
the given endeffectors. endeffs contains the list of ID number for which the Matrix
is derived.\n";
	   
LinkInformation::usage = 
"LinkInformation[infile,outfile] computes the axis of rotation, origin of local
coordinate systems, and link center of gravity for every proper link of the
the system. This information can be used to compute the COG, COG Jacobian, and other
information of the robot in a simple way.\n";	   

OpenGLKinematics::usage = 
"OpenGLKinematics[infile, outfile] derives the C-code to draw a simple OpenGL
based 3D figure of the current movement system. Note that some functions need
to be filled into this code; these functions specify the object that should be
used to display a link. Check help on InvDyn for general rules how to greate
input files.\n";

ParmEst::usage = 
"ParmEst[infile, outfile,gravity] derives the C-code to perform parameter
estimation for inverse dynamics. The same inputs as for the inverse dynamics are
required. The output is a matrix K, and a vector Y. The rows of K have one
data point for the regression analysis, the corresponding Y coeff. is the
output that goes with the data point. K and Y have n_joints rows \n";

GenInertiaMatrix::usage = "Utility to generate a matrix of variables for the
Inertia Matrix; this utility is useful to simplify the input files.\n";

GenMCMVector::usage = "Utility to generate a vector of variables for the
center of mass multiplied with the mass; this utility is useful to simplify 
the input files.\n";

GenMass::usage = "Utility to generate a variables for the mass; 
this utility is useful to simplify the input files.\n"

GenVariables::usage = "Utility to generate variables for joint pos, vel,
and torque; this utility is useful to simplify the input files.\n";

GenBaseVariables::usage = "Utility to generate variables for base pos, vel,
and acc; this utility is useful to simplify the input files.\n";

GenExtForce::usage = "Utility to generate variables for external forces acting
on the link.\n";


GenInertiaMatrixA::usage = "Utility to generate a matrix of variables for the
Inertia Matrix; indexing as array\n";

GenMCMVectorA::usage = "Utility to generate a vector of variables for the
center of mass multiplied with the mass; indexing as array\n";

GenMassA::usage = "Utility to generate a variables for the mass; indexing as array\n";

GenVariablesA::usage = "Utility to generate variables for joint pos, vel,
and torque; indexing as array\n";

GenBaseVariablesA::usage = "Utility to generate variables for base pos, vel,
and acc; this utility is useful to simplify the input files; indexing as array\n";

GenExtForceA::usage = "Utility to generate variables for external forces acting
on the link; indexing as array\n";



GenInertiaMatrixS::usage = "Utility to generate a matrix of variables for the
Inertia Matrix; indexing as CLMC structure";

GenMCMVectorS::usage = "Utility to generate a vector of variables for the
center of mass multiplied with the mass; indexing as CLMC structure";

GenMassS::usage = "Utility to generate a variables for the mass; indexing as CLMC structure";

GenVariablesS::usage = "Utility to generate variables for joint pos, vel,
and torque; indexing as CLMC structure";

GenBaseVariablesS::usage = "Utility to generate variables for base pos, vel,
and acc; this utility is useful to simplify the input files; indexing as CLMC structure\n";

GenExtForceS::usage = "Utility to generate variables for external forces acting
on the link; indexing as CLMC structure.\n";



Begin["ControlTheory`RigidBodyDynamics`Private`"]

(* Switch off some silly warning messages *)
Off[Part::partd]
Off[General::spell]
Off[General::spell1]
		
(* Spatial Arithmitic Functions *)
            
(* Definition of a revolute joint axis:   s:=[a'  (OPxa)']'; 
		where OP is the vector from the origin to the center of the joint, 
		and "a" is the direction of the the rotation as a unit vector. 
		The primes denote the transpose. *)
     
(* spatial coordinate transformation: note vectrans is the translation
		vector that shifts the current origin into the new one; matrot is the
		rotation matrix which turns the current coord.system into the new
		one *)
       
(* a utility to build the spatial matrices *)     		
Reorder[a_,b_,c_,d_]:={Flatten[{a[[1]],b[[1]]}], Flatten[{a[[2]],b[[2]]}],
	Flatten[{a[[3]],b[[3]]}],Flatten[{c[[1]],d[[1]]}], Flatten[{c[[2]],d[[2]]}], 
	Flatten[{c[[3]],d[[3]]}]}
	
(* the spatial transpose *)
STranspose[x_]:=Flatten[{x[[{4,5,6}]],x[[{1,2,3}]]}]

(* a function to create matrices with arbitrary dimensions and uniform content *)
FillMatrix[dims_,val_]:=If[Length[dims]>1,
	Table[FillMatrix[Drop[dims,1],val],{i,1,dims[[1]]}],
	Table[val,{i,1,dims[[1]]}]];
     
(* the Cross product in matrix form *)       				     
CrosstoMatrix[x_]:=
	{{0,-x[[3]],x[[2]]},{x[[3]],0,-x[[1]]},{-x[[2]],x[[1]],0}}
  	
(* finally what we want: the 6x6 transformation matrix *)		
Xtransspatial[matrot_,vectrans_,pristrans_]:=Reorder[matrot,
	DiagonalMatrix[{0,0,0}],
	Transpose[CrosstoMatrix[pristrans]].matrot+matrot.Transpose[CrosstoMatrix[vectrans]],matrot]
               
(* the inverse of Xtransspatial *)
Xtransspatialinv[matrotinv_,vectrans_,pristrans_]:=
	Reorder[matrotinv,DiagonalMatrix[{0,0,0}],
	CrosstoMatrix[vectrans].matrotinv+matrotinv.CrosstoMatrix[pristrans],matrotinv]
						
(* the spatial vector as used for the axis of rotation *)
Sspatial[a_,OP_]:=Join[a,Cross[OP,a]]

(* the spatial velocity, derived from a point with velocity and angular velocity *)
Vel2Spatial[r_,rd_,w_]:=Flatten[{w,rd+CrosstoMatrix[r].w}]
Spatial2Vel[v_,r_]:={v[[{1,2,3}]],v[[{4,5,6}]]-CrosstoMatrix[r].v[[{1,2,3}]]}
			
(* the spatial acceleration, derived from a point with velocity and angular velocity, and corresponding accelerations *)
Acc2Spatial[r_,rd_,rdd_,w_,wd_]:=Flatten[{wd,rdd+CrosstoMatrix[rd].w+CrosstoMatrix[r].wd}]
Spatial2Acc[add_,r_,rd_,w_]:={add[[{1,2,3}]],add[[{4,5,6}]]-CrosstoMatrix[rd].w-CrosstoMatrix[r].add[[{1,2,3}]]}
			
(* recursive definition of joint velocites (startvalue for base=0) *)
Vispatial[viprev_,Xiprev_,s_,thd_]:=Xiprev.viprev + s thd
     
(* recursive definition of joint accelerations (startvalue = +g if
		gravity acts on the base *)
     
CrossSpatial[x_]:= Reorder[CrosstoMatrix[Take[x,3]] ,
	DiagonalMatrix[{0,0,0}],
	CrosstoMatrix[Take[x,-3]],CrosstoMatrix[Take[x,3]] ]
						
Aispatial[aiprev_,Xiprev_,v_,s_,thd_,thdd_]:=
	Xiprev.aiprev+CrossSpatial[v].s thd + s thdd
						
(* spatial inertia matrices: J is the inertia in local coordinates
   but not necessarily in the center of mass system *)
InertiaSpatial[mass_,OP_,J_]:=Reorder[mass CrosstoMatrix[-OP],
	mass IdentityMatrix[3],J,CrosstoMatrix[OP] mass]

(* spatial inertia matrices: this definition can be used alternatively
		if the center of mass coordinate system are to be used in the 
		multiplied form m*vector_cm; this is favorable for parameter
		estimation; note again that J is not in CM coordinates, but
		a constant in the local coordinates. See An et al., page 70 *)
		
InertiaSpatialSpec[m_,mcm_,J_]:={{0, mcm[[3]], -(mcm[[2]]), m, 0, 0}, 
	{-(mcm[[3]]), 0, mcm[[1]], 0, m, 0}, 
	{mcm[[2]], -(mcm[[1]]), 0, 0, 0, m}, {J[[1]][[1]], J[[1]][[2]], J[[1]][[3]], 0, 
	-(mcm[[3]]), mcm[[2]]}, 
	{J[[1]][[2]], J[[2]][[2]], J[[2]][[3]], mcm[[3]], 0, -(mcm[[1]])}, 
	{J[[1]][[3]], J[[2]][[3]], J[[3]][[3]], -(mcm[[2]]), mcm[[1]], 0}}
           
(* the composite spatial inertias without the current inertia at i (to be
		added manually to deal with branches *)
InertiaSpatialComp[Xi1toi_,Jcompi1_,Xitoi1_]:=Xi1toi.Jcompi1.Xitoi1

(* the unit accelerations  fi= Icompi*si *)
Ftransdown[fij1_,Xij1toij_]:=Xij1toij.fij1

(* the elements of the generalized inertia matrix H in Q = H(q) qdd + C(q,qd),
		calculated from projecting the force at joint i onto all joints below this 
		joint. Since H is symmetric, this will result in the full matrix H *)
Hijcomponent[sj_,fij_]:=Flatten[{Take[sj,-3],Take[sj,3]}].fij

(* the net forces for every joint *)
Fnetspatial[J_,a_,v_,S_,fext_]:=J.a + CrossSpatial[v].J.v -Flatten[{S.fext[[{1,2,3}]],S.fext[[{4,5,6}]]}]
      
(* the equation of motion for one joint *)
Ftotspatial[Xitoi1_,ftoti1_]:=Xitoi1.ftoti1
Qjoint[s_,f_]:=Flatten[{Take[s,-3],Take[s,3]}].f

(* functions for articulated body forward dynamics *)
CMisc[v_,s_,thd_]:=CrossSpatial[v].s * thd
PVMisc[v_,J_,S_,fext_,g_]:=CrossSpatial[v].J.v-
	Flatten[{S.fext[[{1,2,3}]],S.fext[[{4,5,6}]]}]-J.Flatten[{{0,0,0},S.g}];
TMisc1[JAj_,hj_,dj_]:=JAj-Outer[Times,hj,STranspose[hj]]/dj
TMisc[Xij_,Xji_,T1_]:=Xij.T1.Xji
HMisc[JA_,s_]:=JA.s
DMisc[s_,h_]:=STranspose[s].h 
PMisc1[pj_,JAj_,cj_,uj_,hj_,dj_]:=pj+JAj.cj+(uj*hj)/dj
PMisc[Xij_,P1_]:=Xij.P1
UMisc[q_,h_,c_,s_,p_]:=q-STranspose[h].c-STranspose[s].p
AispatialAB[aiprev_,Xiprev_,c_,s_,thdd_]:=Xiprev.aiprev + c + s thdd
Thdd[u_,h_,Xprev_,aprev_,d_]:=(u-STranspose[h].Xprev.aprev)/(d)
      
      
(* a short cut to calculate the rotation matrix in Euler angle notation *)
      
RotMat[a_,b_,g_]:=
	{{Cos[b]*Cos[g], Cos[g]*Sin[a]*Sin[b] + Cos[a]*Sin[g], 
	-(Cos[a]*Cos[g]*Sin[b]) + Sin[a]*Sin[g]}, 
	{-(Cos[b]*Sin[g]), Cos[a]*Cos[g] - Sin[a]*Sin[b]*Sin[g], 
	Cos[g]*Sin[a] + Cos[a]*Sin[b]*Sin[g]}, 
	{Sin[b], -(Cos[b]*Sin[a]), Cos[a]*Cos[b]}}
      
RotMatInv[a_,b_,g_]:=
	{{Cos[b]*Cos[g], -(Cos[b]*Sin[g]), Sin[b]}, 
	{Cos[g]*Sin[a]*Sin[b] + Cos[a]*Sin[g], 
	Cos[a]*Cos[g] - Sin[a]*Sin[b]*Sin[g], -(Cos[b]*Sin[a])}, 
	{-(Cos[a]*Cos[g]*Sin[b]) + Sin[a]*Sin[g], 
	Cos[g]*Sin[a] + Cos[a]*Sin[b]*Sin[g], Cos[a]*Cos[b]}}
	
(* conversion from quaternion to rotation matrix and inverse *)

RotMatQuat[q_]:=
	{{-1+2*q[[1]]^2+2*q[[2]]^2,2*(q[[2]]*q[[3]]+q[[1]]q[[4]]),2*(q[[2]]q[[4]]-q[[1]]q[[3]])},
	 {2*(q[[2]]q[[3]]-q[[1]]*q[[4]]),-1+2*q[[1]]^2+2*q[[3]]^2,2*(q[[3]]q[[4]]+q[[1]]q[[2]])},
	 {2*(q[[2]]q[[4]]+q[[1]]*q[[3]]),2*(q[[3]]*q[[4]]-q[[1]]q[[2]]),-1+2*q[[1]]^2+2*q[[4]]^2}}
	 	 
RotMatQuatInv[q_]:=Transpose[RotMatQuat[q]]
   
(* conversion from quaternion velocity to angular velocity *)

Quat2Mat[q_]:={{-q[[2]],-q[[3]],-q[[4]]},
			   { q[[1]], q[[4]],-q[[3]]},
			   {-q[[4]], q[[1]], q[[2]]},
			   { q[[3]],-q[[2]], q[[1]]}}

QuatNorm[q_]:=Sqrt[q.q]

Quat2AngVel[q_,qd_]:=2*Transpose[Quat2Mat[q]].qd
	
AngAcc2Quat[q_,qd_,w_,wd_]:=(Quat2Mat[qd].w+Quat2Mat[q].wd)/2
	

   
(* a special version using sin and cos precomputed *)
      
RotMatSpec[a_]:=
	{{a[[3,2]]*a[[2,2]], a[[1,2]]*a[[3,1]] + a[[1,1]]*a[[3,2]]*a[[2,1]], 
	a[[1,1]]*a[[3,1]] - a[[1,2]]*a[[3,2]]*a[[2,1]]}, 
	{-(a[[3,1]]*a[[2,2]]), a[[1,2]]*a[[3,2]] - 
	a[[1,1]]*a[[3,1]]*a[[2,1]], 
	a[[1,1]]*a[[3,2]] + a[[1,2]]*a[[3,1]]*a[[2,1]]}, 
	{a[[2,1]], -(a[[1,1]]*a[[2,2]]), a[[1,2]]*a[[2,2]]}}
  				
RotMatInvSpec[a_]:=
	{{a[[3,2]]*a[[2,2]], -(a[[3,1]]*a[[2,2]]), a[[2,1]]}, 
	{a[[1,2]]*a[[3,1]] + a[[1,1]]*a[[3,2]]*a[[2,1]], 
	a[[1,2]]*a[[3,2]] - a[[1,1]]*a[[3,1]]*a[[2,1]], 
	-(a[[1,1]]*a[[2,2]])}, {a[[1,1]]*a[[3,1]] - 
	a[[1,2]]*a[[3,2]]*a[[2,1]], 
	a[[1,1]]*a[[3,2]] + a[[1,2]]*a[[3,1]]*a[[2,1]], a[[1,2]]*a[[2,2]]}}
	
(* This function is needed to sort out the dynamics parameters for 
parameter estimation (see An,Atkeson,Hollerbach, page 91), but note
that spatial algebra has the torques in the upper three elements of
a vector, unlike the wrenches in An et al, where the torques are in the
lower three elements. Thus, the matrix below was derived from spatial
vector arithmetic by solving the fnet (see Featherstone page 43) for the
inertial parameters. Additionally, I added 4 terms for friction and spring
terms in 11-14th parameters to be estimated. However, these terms are zero here 
and need to be filled in by the C-program. This actually assumes that
the friction/spring is inside of the actuator or in the actuator hydraulic
supplies, but NOT like a dash-pot between the two links. The latter
would require to transform the force from the friction to all other
proximal links, and this did not work well for our data *)
		
FormKinematicMatrix[v_, a_] :=
   {{a[[4]] - v[[3]]*v[[5]] + v[[2]]*v[[6]], -v[[2]]^2 - v[[3]]^2,
       -a[[3]] + v[[1]]*v[[2]], a[[2]] + v[[1]]*v[[3]], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {a[[5]] + v[[3]]*v[[4]] - v[[1]]*v[[6]], a[[3]] + v[[1]]*v[[2]],
       -v[[1]]^2 - v[[3]]^2, -a[[1]] + v[[2]]*v[[3]], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
     {a[[6]] - v[[2]]*v[[4]] + v[[1]]*v[[5]], -a[[2]] + v[[1]]*v[[3]], 
      a[[1]] + v[[2]]*v[[3]], -v[[1]]^2 - v[[2]]^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, a[[6]] - v[[2]]*v[[4]] + v[[1]]*v[[5]],
       -a[[5]] - v[[3]]*v[[4]] + v[[1]]*v[[6]], a[[1]], 
      a[[2]] - v[[1]]*v[[3]], a[[3]] + v[[1]]*v[[2]], -(v[[2]]*v[[3]]), 
      v[[2]]^2 - v[[3]]^2, v[[2]]*v[[3]], 0, 0, 0, 0},
     {0, -a[[6]] + v[[2]]*v[[4]] - v[[1]]*v[[5]], 0, 
      a[[4]] - v[[3]]*v[[5]] + v[[2]]*v[[6]], v[[1]]*v[[3]], 
      a[[1]] + v[[2]]*v[[3]], -v[[1]]^2 + v[[3]]^2, a[[2]], 
      a[[3]] - v[[1]]*v[[2]], -(v[[1]]*v[[3]]), 0, 0, 0, 0},
     {0, a[[5]] + v[[3]]*v[[4]] - v[[1]]*v[[6]],
       -a[[4]] + v[[3]]*v[[5]] - v[[2]]*v[[6]], 0, -(v[[1]]*v[[2]]), 
      v[[1]]^2 - v[[2]]^2, a[[1]] - v[[2]]*v[[3]], v[[1]]*v[[2]], 
      a[[2]] + v[[1]]*v[[3]], a[[3]], 0, 0, 0, 0}}
  
(* functions which build new symbolic representations but
		which push through zeros and numbers *)
           
ToES[x_]:=ToExpression[ToString[x,OutputForm]]
FixArray[x_]:=ToExpression[StringReplace[ToString[x,CForm],{"["->"","]"->"",","->"","$"->"","-"->"Minus"," "->"","/"->"Div","*"->"Mult","+"->"Plus","."->""}]]
Zt[x_]:=Table[x[[i]],{i,0,Length[x]}];
Ot[x_]:=Table[x[[i]],{i,1,Length[x]}];
       
MakeSimplerVectorCoef[x_,string_,i_,flag_]:=
	If[ !NumberQ[N[x]] , ToES[StringForm[string,i]],
	If[flag==1 (* && N[x]!=0 *), ToES[StringForm[string,i]], N[x]]]

MakeSimplerVector[x_,string_,flag_]:=
	Table[If[ !NumberQ[N[x[[i]]]] , ToES[StringForm[string,i]],
	If[flag==1 (* && N[x[[i]]]!=0 *), ToES[StringForm[string,i]], N[x[[i]]]]],
	{i,1,Length[x]}]

MakeSimplerMatrix[x_,string_,flag_]:= 
	Table[If[ !NumberQ[N[x[[i,j]]]],ToES[StringForm[string,i,j]],
	If[flag==1 (* && N[x[[i,j]]]!=0 *),ToES[StringForm[string,i,j]],N[x[[i,j]]]] ],
	{i,1,Length[x]},{j,1,Length[x[[i]]]}]
		
MakeSinCos[x_]:= 
	{If[NumberQ[N[Sin[x]]],N[Sin[x]],ToES[StringForm["s``",FixArray[x]]]],
	If[NumberQ[N[Cos[x]]],N[Cos[x]],ToES[StringForm["c``",FixArray[x]]]]}
        
MakeSinCosRot[x_]:= 
	{If[NumberQ[N[Sin[x]]],N[Sin[x]],ToES[StringForm["rs``",FixArray[x]]]],
	If[NumberQ[N[Cos[x]]],N[Cos[x]],ToES[StringForm["rc``",FixArray[x]]]]}

DummyLinkQ[x_]:=NumberQ[x] && x==0;
        
      
(* recursive writing to file: resolves all elements in list,
		fn = filename
		symb = list with symbols
		val = values to be assigned to symbols, i.e, symb=val
*)

(* this "hack" allows to use structures as variables in this program *)
FS[x_]:=StringReplace[x,{"$$$$"->".","$$$"->"]","$$"->"].","$"->"["}]

WriteOutput[fn_,symb_,val_]:= Block[{i,j,temp1,temp2},
	j=0;
	Do[ 
		If[ ListQ[symb[[i]]],
			WriteOutput[fn,symb[[i]],val[[i]]],
			If[NumberQ[symb[[i]]] ,j=j+1,
				temp1=FS[ToString[CForm[symb[[i]]]]];
				temp2=FS[ToString[CForm[val[[i]]/.{-1.->-1,1.->1}]]];
				WriteString[fn,StringForm["``=``;\n",temp1,
				temp2]]
			]
		],
		{i,1,Length[symb]}];
		If[j==Length[symb],Null,WriteString[fn,"\n"]];
]
       
(* declaring of variables *)
WriteDeclaration[fn_,prefix_,list_,suffix_]:= Block[{i,temp={},val},
	Do[If[NumberQ[list[[i]]],Null,
		{If[Length[list[[i]]]>0,val=list[[i,1]],val=list[[i]]];
		If[MemberQ[temp,val],Null,{
		WriteString[fn,StringForm["``````;\n",prefix,CForm[val],suffix]];
		temp = Append[temp,val];
	}]}],
	{i,1,Length[list]}];
	WriteString[fn,"\n"];
]

(* reading the file of joint specifications *)
ReadSpecFile[infile_,gravity_,outfile_]:=
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xnj,xuext,base,i,j,xisprismatic},
	
		(* Read the infile *)
		xlorg=ReadList[infile,Expression];
		
		(* search whether there is a base coordinate system *)
		baseID=0;
		For[i=1, i<=Length[xlorg], ++i,If[xlorg[[i,1,2,1]]==0,baseID=i,Null]];
		If[baseID==0,
			{
				base={
					IdentityMatrix[6]*0, (* base spatial inertia in local coordinates *)
					{1,0,0,0,0,0,0},       (*  position in world coordinates *)
					{0,0,0,0,0,0,0,0,0,0}, (*  velocity in world coordinates *)
					{0,0,0,0,0,0,0,0,0,0}, (*  acceleration in world coordinates *)
					{0,0,0,0,0,0},         (*  external forces in world coordinates *)
					{0}                    (*  floating base indicator *)
					};
				storemcm0 = {0,0,0};
				storem0   = 0;
			},
			{
				base={
					InertiaSpatialSpec[xlorg[[baseID,8,2,1]],xlorg[[baseID,7,2]],xlorg[[baseID,6,2]]],
					xlorg[[baseID,9,2,1]], (* spatial position in world coordinates *)
					xlorg[[baseID,9,2,2]], (* spatial velocity in world coordinates *)
					xlorg[[baseID,9,2,3]], (* spatial acceleration in world coordinates *)
					xlorg[[baseID,10,2]],  (* external forces in world coordinates *)
					xlorg[[baseID,2,2,1]]  (* floating base indicator *)
					};
				storemcm0 = xlorg[[baseID,7,2]];
				storem0   = xlorg[[baseID,8,2,1]];
				xlorg = Drop[xlorg,{baseID}]; (* remove the base specs from the list *)
			}
		];
		
		(* now proceed with extracting info from xlorg as if there is no base *)
		xl=Transpose[xlorg];
		
		(* need to guarantee that distal joints come AFTER more proximal joints
		   in the array sequence *)
		xID     = Flatten[Transpose[xl[[1]]][[2]]]; (* the ID number *)
		xsuc    = Transpose[xl[[5]]][[2]]; (* ID number of following joints *)
		xnj     = Length[xID];
		
		(* build the predecessor list from the successor list: note, that there
				can only be one predecessor *)
		xpred   = Table[
			{n=1;
				Do[
					If[MemberQ[xsuc[[j]],xID[[i]]],res=xID[[j]],
						If[j==xnj,If[n==xnj,res=0,Null],n=n+1]],
					{j,1,xnj}];
					res},
				{i,1,xnj}];
		xpred = Flatten[xpred];
		
		(* generate a correct sequence in the array: start with all joints having
		   the "0" predecessor and just follow their successors until the chain
		   terminates *)
		   
		xxsuc = xsuc;
		Do[ If[xxsuc[[i,j]]==xID[[n]],xsuc[[i,j]]=n,Null],{n,1,xnj},{i,1,xnj},
			{j,1,Length[xsuc[[i]]]}];
		xID=Table[i,{i,1,xnj}];
		   
		FindSucessors[n_]:=Block[{j},
			If[Length[xsuc[[n]]]==0,{n},
				Join[{n},Table[FindSucessors[xsuc[[n,j]]],{j,1,Length[xsuc[[n]]]}]]
			]];
		
		xorder = {};
		Do[ If[xpred[[i]]==0,xorder=Join[xorder,Flatten[FindSucessors[i]]],Null],{i,1,xnj}];
		
		(* re-order the xlorg array, and start over. Now the DOFs are in such a sequence
		   that it is guaranteed that DOFs proximal to the base are filled in before the
		   DOFS that are distal *)
		   
		xl=Transpose[xlorg[[xorder]]];

		(* Extract the relevant arrays *) 
		xID     =Flatten[Transpose[xl[[1]]][[2]]]; (* the ID number *)
		xax     =Transpose[xl[[2]]][[2]]; (* thejoint  rotation axis as vector*)
		xtrans  =Transpose[xl[[3]]][[2]]; (* the translation to this coord.sys *)
		xrot    =Transpose[xl[[4]]][[2]]; (* the rotation to this coord.sys *)
		xsuc    =Transpose[xl[[5]]][[2]]; (* ID number of following joints *)
		xinert  =Transpose[xl[[6]]][[2]]; (* variables for inertia tensor *)
		xmcm    =Transpose[xl[[7]]][[2]]; (* variables for mass * center of mass vec. *)
		xm      =Flatten[Transpose[xl[[8]]][[2]]]; (* variable for mass *)
		xj      =Transpose[xl[[9]]][[2]]; (* variables for joint angle *)
		xuext   =Transpose[xl[[10]]][[2]]; (* variables for external forces *)
					  		
		(* the number of joints we have *)
		xnj      = Length[xj];
		
		(* Replace the ID numbers by an internal ID number such that
		   ID number correspond to indices into the array *)

		xxsuc = xsuc;
		xIDorg = xID;
		xIDorg[[0]]  = 0;
		Do[ If[xxsuc[[i,j]]==xID[[n]],xsuc[[i,j]]=n,Null],{n,1,xnj},{i,1,xnj},
			{j,1,Length[xsuc[[i]]]}];
		xID=Table[i,{i,1,xnj}];
		
		(* build the predecessor list from the successor list: note, that there
				can only be one predecessor *)
		xpred   = Table[
			{n=1;
				Do[
					If[MemberQ[xsuc[[j]],xID[[i]]],res=xID[[j]],
						If[j==xnj,If[n==xnj,res=0,Null],n=n+1]],
					{j,1,xnj}];
					res},
				{i,1,xnj}];
		xpred = Flatten[xpred];
		

		(* add the 0 array element to xsuc that indicates all the DOFs that follow the base *)
		xsuc[[0]]={};
		Do[
		If[xpred[[i]]==0,xsuc[[0]]=Append[xsuc[[0]],i],Null,Null],
		{i,1,xnj}];
		
		(* add various base information *)
		xmcm[[0]] = storemcm0;
		xm[[0]]   = storem0;
		xID[[0]]  = 0;
		xIDorg[[0]] = 0;
			  		
		(* make sure the rotation axis are unit length, and the axis is aligned with one one coordinate axis (this could be relaxed at some point) *)
		Do[
		   If[Norm[xax[[i]]] == 0,Null,
			If[Sign[xax[[i]]].Sign[xax[[i]]] == 1,Null,Print["Joint axis ",xIDorg[i]," can only have ONE nonzero element"],error20],
			error20],
		{i,1,xnj}];
		xax[[0]] = {0,0,0,0,0,0};

		(* distinguish between revolute and prismatic ojoints *)
		xisprismatic=Table[0,{i,1,xnj}];
		Do[
		If[Length[xax[[i]]]>3,{If[xax[[i,{4,5,6}]].xax[[i,{4,5,6}]] > 0, xisprismatic[[i]]=1,Null,Null]},xax[[i]]=Flatten[{xax[[i]],0,0,0}],Null],
		{i,1,xnj}];
		xisprismatic[[0]]=0;

		(* write the prismatic joint  indicator *)
		file    = StringJoin[outfile,"_Prismatic_Joints.h"];
		OpenWrite[file];
		Do[
		If[DummyLinkQ[xj[[i,1]]],Null,WriteString[file,StringForm["prismatic_joint_flag[``]=``;\n",xIDorg[[i]],xisprismatic[[i]]]],{i,1,xnj}],
		{i,1,xnj}];

		(* write the predecssor array *)
		WriteString[file,StringForm["\n"]];
		Do[If[DummyLinkQ[xj[[i,1]]],Null,WriteString[file,StringForm["jointPredecessor[``]=``;\n",xIDorg[[i]],xIDorg[[xpred[[i]]]]]]],{i,1,xnj}];

		Close[file];


		(* write the floating base indicator *)
		file    = StringJoin[outfile,"_Floating_Base.h"];
		OpenWrite[file];
		WriteString[file,StringForm["const int floating_base_flag=``;\n",base[[6]]]];
		Close[file];
		
		(* the final result of this function *)
		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic}
	]
	
(* utility function for reading from definition file *)
GenInertiaMatrix[name_,ID_]:= Block[{dum,i},
	dum=Table[ ToExpression[ToString[StringForm["````[[``,``]]",name,ID,i,j]]],{i,1,3},{j,1,3}];
	Do[dum[[j,i]]=dum[[i,j]],{i,1,3},{j,i,3}];
	dum
	]

GenMCMVector[name_,ID_]:= Block[{dum,i},
	Table[ ToExpression[ToString[StringForm["````[[``]]",name,ID,i]]],{i,1,3}]
	]

GenMass[name_,ID_]:= Block[{dum,i},
	{ToExpression[ToString[StringForm["````",name,ID]]]}
	]

GenVariables[name_,ID_]:= Block[{dum,i},
	{ToExpression[ToString[StringForm["````",name,ID]]],
	ToExpression[ToString[StringForm["````d",name,ID]]],
	ToExpression[ToString[StringForm["````dd",name,ID]]],
	ToExpression[ToString[StringForm["````u",name,ID]]],
	ToExpression[ToString[StringForm["````uex",name,ID]]]}
	]
	
GenExtForce[name_,ID_]:= Block[{dum,i},
	{ToExpression[ToString[StringForm["````fx",name,ID]]],
	ToExpression[ToString[StringForm["````fy",name,ID]]],
	ToExpression[ToString[StringForm["````fz",name,ID]]],
	ToExpression[ToString[StringForm["````ta",name,ID]]],
	ToExpression[ToString[StringForm["````tb",name,ID]]],
	ToExpression[ToString[StringForm["````tg",name,ID]]]}
	]
	
GenBaseVariables[name_,ID_]:= Block[{dum,i},
	{{
	ToExpression[ToString[StringForm["````q0",name,ID]]],
	ToExpression[ToString[StringForm["````q1",name,ID]]],
	ToExpression[ToString[StringForm["````q2",name,ID]]],
	ToExpression[ToString[StringForm["````q3",name,ID]]],
	ToExpression[ToString[StringForm["````x",name,ID]]],
	ToExpression[ToString[StringForm["````y",name,ID]]],
	ToExpression[ToString[StringForm["````z",name,ID]]]
	},{
	ToExpression[ToString[StringForm["````q0d",name,ID]]],
	ToExpression[ToString[StringForm["````q1d",name,ID]]],
	ToExpression[ToString[StringForm["````q2d",name,ID]]],
	ToExpression[ToString[StringForm["````q3d",name,ID]]],
	ToExpression[ToString[StringForm["````xd",name,ID]]],
	ToExpression[ToString[StringForm["````yd",name,ID]]],
	ToExpression[ToString[StringForm["````zd",name,ID]]],
	ToExpression[ToString[StringForm["````ad",name,ID]]],
	ToExpression[ToString[StringForm["````bd",name,ID]]],
	ToExpression[ToString[StringForm["````gd",name,ID]]]
	},{
	ToExpression[ToString[StringForm["````q0dd",name,ID]]],
	ToExpression[ToString[StringForm["````q1dd",name,ID]]],
	ToExpression[ToString[StringForm["````q2dd",name,ID]]],
	ToExpression[ToString[StringForm["````q3dd",name,ID]]],
	ToExpression[ToString[StringForm["````xdd",name,ID]]],
	ToExpression[ToString[StringForm["````ydd",name,ID]]],
	ToExpression[ToString[StringForm["````zdd",name,ID]]],
	ToExpression[ToString[StringForm["````add",name,ID]]],
	ToExpression[ToString[StringForm["````bdd",name,ID]]],
	ToExpression[ToString[StringForm["````gdd",name,ID]]]
	}}
	]
	
(* the same function, but now they generate arrays of variables *)

GenInertiaMatrixA[name_,ID_]:= Block[{dum,i},
	dum=Table[ ToExpression[ToString[StringForm["``[[``,``,``]]",name,ID,i,j]]],{i,1,3},{j,1,3}];
	Do[dum[[j,i]]=dum[[i,j]],{i,1,3},{j,i,3}];
	dum
	]

GenMCMVectorA[name_,ID_]:= Block[{dum,i},
	Table[ ToExpression[ToString[StringForm["``[[``,``]]",name,ID,i]]],{i,1,3}]
	]

GenMassA[name_,ID_]:= Block[{dum,i},
	{ToExpression[ToString[StringForm["``[[``]]",name,ID]]]}
	]

GenVariablesA[name_,ID_]:= Block[{dum,i},
	{ToExpression[ToString[StringForm["``[[``]]",name,ID]]],
	ToExpression[ToString[StringForm["``d[[``]]",name,ID]]],
	ToExpression[ToString[StringForm["``dd[[``]]",name,ID]]],
	ToExpression[ToString[StringForm["``u[[``]]",name,ID]]],
	ToExpression[ToString[StringForm["``uex[[``]]",name,ID]]]}
	]

GenExtForceA[name_,ID_]:= Block[{dum,i},
	{ToExpression[ToString[StringForm["``f[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``f[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``f[[``,3]]",name,ID]]],
	ToExpression[ToString[StringForm["``t[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``t[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``t[[``,3]]",name,ID]]]}
	]

GenBaseVariablesA[name_,ID_]:= Block[{dum,i},
	{{
	ToExpression[ToString[StringForm["``q[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``q[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``q[[``,3]]",name,ID]]],
	ToExpression[ToString[StringForm["``q[[``,4]]",name,ID]]],
	ToExpression[ToString[StringForm["``x[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``x[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``x[[``,3]]",name,ID]]]
	},{
	ToExpression[ToString[StringForm["``qd[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``qd[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``qd[[``,3]]",name,ID]]],
	ToExpression[ToString[StringForm["``qd[[``,4]]",name,ID]]],
	ToExpression[ToString[StringForm["``xd[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``xd[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``xd[[``,3]]",name,ID]]],
	ToExpression[ToString[StringForm["``ad[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``ad[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``ad[[``,3]]",name,ID]]]
	},{
	ToExpression[ToString[StringForm["``qdd[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``qdd[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``qdd[[``,3]]",name,ID]]],
	ToExpression[ToString[StringForm["``qdd[[``,4]]",name,ID]]],
	ToExpression[ToString[StringForm["``xdd[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``xdd[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``xdd[[``,3]]",name,ID]]],
	ToExpression[ToString[StringForm["``add[[``,1]]",name,ID]]],
	ToExpression[ToString[StringForm["``add[[``,2]]",name,ID]]],
	ToExpression[ToString[StringForm["``add[[``,3]]",name,ID]]]
	}}
	]
	

(* the same function, but now they generate arrays of structures of variables
   according to the CLMC convention *)

GenInertiaMatrixS[name_,ID_,si_]:= Block[{dum,i},
	dum=Table[ ToExpression[ToString[StringForm["``$``$$inertia[[``,``]]",name,ID,i,j]]],{i,si,si+2},{j,si,si+2}];
	Do[dum[[j,i]]=dum[[i,j]],{i,si,si+2},{j,i,si+2}];
	dum
	]

GenMCMVectorS[name_,ID_,si_]:= Block[{dum,i},
	Table[ ToExpression[ToString[StringForm["``$``$$mcm[[``]]",name,ID,i]]],{i,si,si+2}]
	]

GenMassS[name_,ID_]:= Block[{dum,i},
	{ToExpression[ToString[StringForm["``$``$$m",name,ID]]]}
	]

GenVariablesS[name_,ID_]:= Block[{dum,i},
	{ToExpression[ToString[StringForm["``$``$$th",name,ID]]],
	ToExpression[ToString[StringForm["``$``$$thd",name,ID]]],
	ToExpression[ToString[StringForm["``$``$$thdd",name,ID]]],
	ToExpression[ToString[StringForm["``$``$$u",name,ID]]],
	ToExpression[ToString[StringForm["``$``$$uex",name,ID]]]}
	]

GenExtForceS[name_,ID_]:= Block[{dum,i},
	{ToExpression[ToString[StringForm["``$``$$f[[1]]",name,ID]]],
	ToExpression[ToString[StringForm["``$``$$f[[2]]",name,ID]]],
	ToExpression[ToString[StringForm["``$``$$f[[3]]",name,ID]]],
	ToExpression[ToString[StringForm["``$``$$t[[1]]",name,ID]]],
	ToExpression[ToString[StringForm["``$``$$t[[2]]",name,ID]]],
	ToExpression[ToString[StringForm["``$``$$t[[3]]",name,ID]]]}
	]

GenBaseVariablesS[nameC_,nameO_,ID_]:= Block[{dum,i},
	{{
	ToExpression[ToString[StringForm["``$``$$q[[1]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$q[[2]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$q[[3]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$q[[4]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$x[[1]]",nameC,ID]]],
	ToExpression[ToString[StringForm["``$``$$x[[2]]",nameC,ID]]],
	ToExpression[ToString[StringForm["``$``$$x[[3]]",nameC,ID]]]
	},{
	ToExpression[ToString[StringForm["``$``$$qd[[1]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$qd[[2]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$qd[[3]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$qd[[4]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$xd[[1]]",nameC,ID]]],
	ToExpression[ToString[StringForm["``$``$$xd[[2]]",nameC,ID]]],
	ToExpression[ToString[StringForm["``$``$$xd[[3]]",nameC,ID]]],
	ToExpression[ToString[StringForm["``$``$$ad[[1]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$ad[[2]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$ad[[3]]",nameO,ID]]]
	},{
	ToExpression[ToString[StringForm["``$``$$qdd[[1]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$qdd[[2]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$qdd[[3]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$qdd[[4]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$xdd[[1]]",nameC,ID]]],
	ToExpression[ToString[StringForm["``$``$$xdd[[2]]",nameC,ID]]],
	ToExpression[ToString[StringForm["``$``$$xdd[[3]]",nameC,ID]]],
	ToExpression[ToString[StringForm["``$``$$add[[1]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$add[[2]]",nameO,ID]]],
	ToExpression[ToString[StringForm["``$``$$add[[3]]",nameO,ID]]]
	}}
	]
	
(* function to faciliate generating subprograms for faster and easier compilation *)

WriteFunctionStart[fid_,name_,num_]:= Block[{},
	WriteString[fid,"\nvoid\n"];
	WriteString[fid,name];
	WriteString[fid,ToString[StringForm["func``(void)\n{\n",num]]];
	]

WriteFunctionEnd[fid_,name_,num_]:= Block[{},
	WriteString[fid,"\n}\n\n"];
	]

WriteFunctionExe[fid_,name_,num_]:= Block[{},
	WriteString[fid,ToString[StringForm["``func``();\n\n",name,num]]];
	]

WriteFunctionDec[fid_,name_,num_]:= Block[{},
	WriteString[fid,ToString[StringForm[""]]]; (* this is obsolete *)
	]

(************************* Inverse Dynamics  **********************)
(************************* Inverse Dynamics  **********************)
(************************* Inverse Dynamics  **********************)

(* to re-use the inverse dynamics more efficiently for the forward
		dynamics, it comes in two definitions *)
		
InvDyn[infile_,outfile_,gravity_]:= Block[{temp},
	temp=StringJoin[outfile,"_InvDyn"];
	xl = ReadSpecFile[infile,gravity,outfile];
	(* this simple form of inv.dyn. does not include external forces 
	   in order to maintain compatibility of this function, and we DO
	   assume a fixed base *)
	xl[[13]] = xl[[13]]*0;      (* zero external forces *)
	xl[[14]]   = xl[[14]]*0;    (* zero the base variables *)
	xl[[14,2,1]] = 1;           (* add the quaternion element *)
	InvDynSpec[xl,temp,gravity]]

(* this special version takes the base vel and acc. into account and also computes
   the force/torque vectors for the base, i.e., the ground reaction force/torques
   at the base *)
InvDynNE[infile_,outfile_,gravity_]:= Block[{temp},
	temp=StringJoin[outfile,"_InvDynNE"];
	xl = ReadSpecFile[infile,gravity,outfile];
	InvDynSpec[xl,temp,gravity]]

InvDynSpec[speclist_,outfile_,gravity_]:=

	(* now we start calculating the inverse dynamics *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = speclist;
			  		
		(* the output file *)
		file    = StringJoin[outfile,"_math.h"];
		filedec = StringJoin[outfile,"_declare.h"];
		filefun = StringJoin[outfile,"_functions.h"];
		OpenWrite[file];
		OpenWrite[filedec];
		OpenWrite[filefun];
			  		
		(* fix the numerics *)
		rep = {1.->1, -1.-> -1, 0.->0, 2.->2, 3.->3};
		
		(* a counter for the subprograms that we generate *)
		countfun=0;
			  		
		(* Note: ox... variables are output for C -- they are matched by
			a x... variable which contains the symbol which should be
			assigned, i.e, x... = ox... will be a proper C assignment *)
			  		
		(* precompute the sin and cos for each joint, but zero out prismatic joints *)
		oxjsincos = Table[Flatten[{Sin[xj[[i,1]]],Cos[xj[[i,1]]]}(1-xisprismatic[[i]])],{i,1,xnj}];
		xjsincos = Table[MakeSinCos[xj[[i,1]]],{i,1,xnj}];
		WriteString[file,"/* sine and cosine precomputation */\n"];
		WriteOutput[file,xjsincos,oxjsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xjsincos],""];

			  		
		(* precompute the sin and cos for the rotation matrices *)
		oxrotsincos = 
			Table[Flatten[N[{Sin[xrot[[i,j]]],Cos[xrot[[i,j]]]}]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}] /. rep;
		xrotsincos = Table[MakeSinCosRot[xrot[[i,j]]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}] /. rep;
		WriteString[file,"/* rotation matrix sine and cosine precomputation */\n"];
		WriteOutput[file,xrotsincos,oxrotsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xrotsincos],""];
			  		   
		(* calculate the 3D rotation matrices *)
		oxS = Table[
			RotMatSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error1],
			{j,1,3}]].
			RotMatSpec[xrotsincos[[i]]],
			{i,1,xnj}];
		xS = Table[
			MakeSimplerMatrix[oxS[[i]],
				ToString[StringForm["S``````",xID[[i]],xpred[[i]],"[[``,``]]"]],0],
				{i,1,xnj}];
				
		(* rotation matrix for floating base *)
		oxS[[0]] = RotMatQuat[base[[2,{1,2,3,4}]]];
		xS[[0]] = MakeSimplerMatrix[oxS[[0]],
				ToString[StringForm["S``````",0,0,"[[``,``]]"]],0];
				

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* rotation matrices */\n"];
		WriteOutput[filefun,Zt[xS],Zt[oxS]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xS]],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];		
			  		   
		(* need the inverse matrices as well *)
		oxSinv = Table[
			RotMatInvSpec[xrotsincos[[i]]].
			RotMatInvSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error2],
			{j,1,3}]],
			{i,1,xnj}] /. rep;
		xSinv = Table[
			MakeSimplerMatrix[oxSinv[[i]],
			ToString[StringForm["Si``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0],
			{i,1,xnj}];
		
		(* rotation matrix for floating base *)
		oxSinv[[0]] = RotMatQuatInv[base[[2,{1,2,3,4}]]] /. rep;
		xSinv[[0]] = MakeSimplerMatrix[oxSinv[[0]],
			ToString[StringForm["Si``````",0,0,"[[``,``]]"]],0];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* inverse rotation matrices */\n"];
		WriteOutput[filefun,Zt[xSinv],Zt[oxSinv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xSinv]],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
		

		(* for converting the external forces into local coordinates, the rotation
		   matrix from global to local coordinates of every joint is needed. This is
		   easily obtained from multiplying the S matrices with each other *)
		   
		xSG = {}; 
		oxSG = {}; 
		xSG[[0]] = xS[[0]]; (* note:changing the zero elements destroys the "list" identifier *)
		Do[
		{
			oxSG=Append[oxSG,xS[[i]].xSG[[xpred[[i]]]]] /. rep;
			xSG=Append[xSG,MakeSimplerMatrix[oxSG[[i]],
				ToString[StringForm["SG``0``",xID[[i]],"[[``,``]]"]],0]]
		},
		{i,1,xnj}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* rotation matrices from global to link coordinates */\n"];
		WriteOutput[filefun,xSG,oxSG];
		WriteDeclaration[filedec,"double  ",Flatten[Ot[xSG]],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];			  		   				
					  		   
			  		
		(* build the spatial rotation matrices *)
		xX = Table[
			Xtransspatial[xS[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]],
			{i,1,xnj}];
		xXinv = Table[
			Xtransspatialinv[xSinv[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]],
			{i,1,xnj}];
			
		(* add the floating base in the zero element: the translation part is zeroed out since
           the transformation from base to world has no rigid link and should just be a rotation,
           and not a spatial rotation *)
		xX[[0]]    = Xtransspatial[xS[[0]],0*base[[2,{5,6,7}]],{0,0,0}];
		xXinv[[0]] = Xtransspatialinv[xSinv[[0]],0*base[[2,{5,6,7}]],{0,0,0}];
			  		
		(* build the spatial rotation vectors *)
		xs = Table[
			xax[[i]],
			{i,1,xnj}];
			       		  		
		(* build the spatial inertia matrices *)
		xJ = Table[
			InertiaSpatialSpec[xm[[i]],xmcm[[i]],xinert[[i]]],
			{i,1,xnj}];
			
		(* the base inertia can be copied from the base variable *)
		xJ[[0]] = base[[1]];
			       		  		
		(* build the velocity vectors *)
		xv = {}; 
		oxv = {}; 
				   
		xvbase      = Vel2Spatial[base[[2,{5,6,7}]]*0,base[[3,{5,6,7}]],base[[3,{8,9,10}]]];
		xX0spec     = xX[[0]];
		xX0spec[[{4,5,6},{1,2,3}]]=0; (* this eliminates the translation part: actually redundant as xX[[0]] has been fixed abo *)
		oxv[[0]]    = xX0spec.xvbase;
		xv[[0]]     = MakeSimplerVector[oxv[[0]],ToString[StringForm["v````",0,"[[``]]"]],0];
		
		Do[
			{oxv=Append[oxv,Vispatial[xv[[xpred[[i]]]],xX[[i]],xs[[i]],xj[[i,2]]]] /. rep;
			xv=Append[xv,MakeSimplerVector[oxv[[i]],
			ToString[StringForm["v````",xID[[i]],"[[``]]"]],0]]},
			{i,1,xnj}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* velocity vectors */\n"];
		WriteOutput[filefun,Zt[xv],Zt[oxv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xv]],"[6+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
		
		
		(* build the acceleration vectors *)
		xa = {}; 
		oxa = {}; 
		(* This allows full accelerations, although most methods won't need it. Note that this is a simplified
        verion of Acc2Spatial, which leaves out all cross products. *)
	    xabase = Flatten[{base[[4,{8,9,10}]],base[[4,{5,6,7}]]-gravity}];

		oxa[[0]] = xX0spec.xabase;
		xa[[0]] = MakeSimplerVector[oxa[[0]],ToString[StringForm["a````",xID[[0]],"[[``]]"]],0];
		Do[
			{oxa=Append[oxa,Aispatial[xa[[xpred[[i]]]],xX[[i]],xv[[i]],xs[[i]],xj[[i,2]],xj[[i,3]]]] /. rep;
			xa=Append[xa,MakeSimplerVector[oxa[[i]],
			ToString[StringForm["a````",xID[[i]],"[[``]]"]],0]]},
			{i,1,xnj}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];
		WriteString[filefun,"/* acceleration vectors */\n"];
		WriteOutput[filefun,Zt[xa],Zt[oxa]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xa]],"[6+1]"];
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];			  		   
			  		   
		(* build the net forces, including the base force needed for composite body forward dynamics.
		   Note that external forces are accumulated separately to facilitate computations in the
		   composite body forward dynamics *)
		xfnet = {}; 
		oxfnet = {}; 
		xfex = {}; 
		oxfex = {}; 
		Do[
			{
			oxfnet=Append[oxfnet,Fnetspatial[xJ[[i]],xa[[i]],xv[[i]],xSG[[i]],0*xuext[[i]]]] /. rep;
			xfnet=Append[xfnet,MakeSimplerVector[oxfnet[[i]],
				ToString[StringForm["fnet````",xID[[i]],"[[``]]"]],0]];
			oxfex=Append[oxfex,Fnetspatial[xJ[[i]],0*xa[[i]],0*xv[[i]],xSG[[i]],xuext[[i]]]] /. rep;
			xfex=Append[xfex,MakeSimplerVector[oxfex[[i]],
				ToString[StringForm["fex````",xID[[i]],"[[``]]"]],0]]
			},
			{i,1,xnj}];
			
		xuext[[0]]=base[[5]];
	
		oxfnet[[0]] = Fnetspatial[xJ[[0]],xa[[0]],xv[[0]],xSG[[0]],0*xuext[[0]]] /. rep;
		xfnet[[0]]  = MakeSimplerVector[oxfnet[[0]],ToString[StringForm["fnet````",xID[[0]],"[[``]]"]],0];

		oxfex[[0]] = Fnetspatial[xJ[[0]],0*xa[[0]],0*xv[[0]],xSG[[0]],xuext[[0]]] /. rep;
		xfex[[0]]  = MakeSimplerVector[oxfex[[0]],ToString[StringForm["fex````",xID[[0]],"[[``]]"]],0];

		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];
		WriteString[filefun,"/* net forces and external forces for each joint */\n"];
		WriteOutput[filefun,Zt[xfnet],Zt[oxfnet]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xfnet]],"[6+1]"];
		WriteOutput[filefun,Zt[xfex],Zt[oxfex]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xfex]],"[6+1]"];
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];		
					  		   
		(* build the total forces *)
		xf = FillMatrix[{xnj,6},0]; (* need xf initialized since Do-loop runs backward and indexes into it *)
		oxf = xfnet; (* all xf have xfnet as additive component *)
		xfext = FillMatrix[{xnj,6},0]; (* the same for external forces *)
		oxfext = xfex; (* all xfext have xfex as additive component *)
		Do[
			{
			Do[ oxf[[i]] = oxf[[i]] + Ftotspatial[xXinv[[xsuc[[i,j]]]],xf[[xsuc[[i,j]]]]] /. rep,
				{j,1,Length[xsuc[[i]]]}];
			xf[[i]]=MakeSimplerVector[oxf[[i]],ToString[StringForm["f````",xID[[i]],"[[``]]"]],0];
			Do[ oxfext[[i]] = oxfext[[i]] + Ftotspatial[xXinv[[xsuc[[i,j]]]],xfext[[xsuc[[i,j]]]]] /. rep,
				{j,1,Length[xsuc[[i]]]}];
			xfext[[i]]=MakeSimplerVector[oxfext[[i]],ToString[StringForm["fext````",xID[[i]],"[[``]]"]],0]
			},
			{i,xnj,0,-1}];
			
		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];
		WriteString[filefun,"/* total forces and external forces for each joint */\n"];
		WriteOutput[filefun,Reverse[Zt[xf]],Reverse[Zt[oxf]]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xf]],"[6+1]"];
		WriteOutput[filefun,Reverse[Zt[xfext]],Reverse[Zt[oxfext]]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xfext]],"[6+1]"];
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];

        (* compute the base force/torque vector in world coordinates *)
		WriteString[file,"/* force/torque of base in world coordinates */\n"];
		oxfbase = Ftotspatial[xXinv[[0]],xf[[0]]] /. rep;		
	    xfbase  = MakeSimplerVector[oxfbase,ToString[StringForm["fbase``","[[``]]"]],1];
  	  WriteOutput[file,xfbase,oxfbase];

		(* and finally the inverse dynamics joint torques include all external torques *)
		xq = Table[xj[[i,4]],{i,1,xnj}];
		(* if we use CLMC structures, the u must become uff for inv.dyn *)
		xq=ToExpression[StringReplace[ToString[xq,OutputForm] ,"$$u"->"$$uff"]];
		oxq = Table[Qjoint[xs[[i]],xf[[i]]+xfext[[i]]-xj[[i,5]]],{i,1,xnj}];
		WriteString[file,"/* inverse dynamics torques */\n"];
		WriteOutput[file,xq,oxq];
		
		(* compute torques due to external forces, such that this info can be used
		   by the composite body forward dynamics to obtain the proper C+G terms
		   Note: these external force are in the internal numbering scheme *)
		oxqext = Table[Qjoint[xs[[i]],xfext[[i]]-xj[[i,5]]],{i,1,xnj}];
		xqext  = MakeSimplerVector[oxqext,ToString[StringForm["qext``","[[``]]"]],1];
		WriteString[file,"/* torques due to external forces */\n"];
		WriteOutput[file,xqext,oxqext];
		WriteDeclaration[filedec,"double  ",Flatten[xqext],ToString[StringForm["[``+1]",Length[xqext]]]];
			  		
		Close[file];
		Close[filedec];
		Close[filefun];
	]

(************************* Forward Dynamics Articulate Body  **********************)
(************************* Forward Dynamics Articulate Body  **********************)
(************************* Forward Dynamics Articulate Body  **********************)

ForDynArt[infile_,outfile_,gravity_]:=

	(* now we start calculating the forward dynamics *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,gravity,outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;
  					  		
		(* the output file *)
		temp=StringJoin[outfile,"_ForDynArt"];
		file    = StringJoin[temp,"_math.h"];
		filedec = StringJoin[temp,"_declare.h"];
		filefun = StringJoin[temp,"_functions.h"];
		OpenWrite[file];
		OpenWrite[filedec];
		OpenWrite[filefun];
			  		
		(* fix the numerics *)
		rep = {1.->1, -1.-> -1, 0.->0, 2.->2, 3.->3};
		
		(* a counter for the subprograms that we generate *)
		countfun=0;
			  		
		(* Note: ox... variables are output for C -- they are matched by
			a x... variable which contains the symbol which should be
			assigned, i.e, x... = ox... will be a proper C assignment *)
			  		
		(* precompute the sin and cos for each joint *)
		oxjsincos = Table[Flatten[{Sin[xj[[i,1]]],Cos[xj[[i,1]]]}(1-xisprismatic[[i]])],{i,1,xnj}];
		xjsincos = Table[MakeSinCos[xj[[i,1]]],{i,1,xnj}];
		WriteString[file,"/* sine and cosine precomputation */\n"];
		WriteOutput[file,xjsincos,oxjsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xjsincos],""];

			  		
		(* precompute the sin and cos for the rotation matrices *)
		oxrotsincos = 
			Table[Flatten[N[{Sin[xrot[[i,j]]],Cos[xrot[[i,j]]]}]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}] /. rep;
		xrotsincos = Table[MakeSinCosRot[xrot[[i,j]]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}] /. rep;

		WriteString[file,"/* rotation matrix sine and cosine precomputation */\n"];
		WriteOutput[file,xrotsincos,oxrotsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xrotsincos],""];
									
			  		   
		(* calculate the 3D rotation matrices *)
		oxS = Table[
			RotMatSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error1],
			{j,1,3}]].
			RotMatSpec[xrotsincos[[i]]],
			{i,1,xnj}];
		xS = Table[
			MakeSimplerMatrix[oxS[[i]],
				ToString[StringForm["S``````",xID[[i]],xpred[[i]],"[[``,``]]"]],0],
				{i,1,xnj}];
				
		(* rotation matrix for floating base *)
		oxS[[0]] = RotMatQuat[base[[2,{1,2,3,4}]]];
		xS[[0]] = MakeSimplerMatrix[oxS[[0]],
				ToString[StringForm["S``````",0,0,"[[``,``]]"]],0];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* rotation matrices */\n"];
		WriteOutput[filefun,Zt[xS],Zt[oxS]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xS]],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];		
			  		   
				
		(* need the inverse matrices as well *)
		oxSinv = Table[
			RotMatInvSpec[xrotsincos[[i]]].
			RotMatInvSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error2],
			{j,1,3}]],
			{i,1,xnj}] /. rep;
		xSinv = Table[
			MakeSimplerMatrix[oxSinv[[i]],
			ToString[StringForm["Si``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0],
			{i,1,xnj}];
		
		(* rotation matrix for floating base *)
		oxSinv[[0]] = RotMatQuatInv[base[[2,{1,2,3,4}]]] /. rep;
		xSinv[[0]] = MakeSimplerMatrix[oxSinv[[0]],
			ToString[StringForm["Si``````",0,0,"[[``,``]]"]],0];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* inverse rotation matrices */\n"];
		WriteOutput[filefun,Zt[xSinv],Zt[oxSinv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xSinv]],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
		

		(* for converting the external forces and gravity into local coordinates, the rotation
		   matrix from global to local coordinates of every joint is needed. This is
		   easily obtained from multiplying the S matrices with each other *)
		   
		xSG = {}; 
		oxSG = {}; 
		xSG[[0]] = xS[[0]]; (* note:changing the zero elements destroys the "list" identifier *)
		Do[
		{
			oxSG=Append[oxSG,xS[[i]].xSG[[xpred[[i]]]]] /. rep;
			xSG=Append[xSG,MakeSimplerMatrix[oxSG[[i]],
				ToString[StringForm["SG``0``",xID[[i]],"[[``,``]]"]],0]]
		},
		{i,1,xnj}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* rotation matrices from global to link coordinates */\n"];
		WriteOutput[filefun,xSG,oxSG];
		WriteDeclaration[filedec,"double  ",Flatten[Ot[xSG]],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];			  		   				
					  		   
			  		
		(* build the spatial rotation matrices *)
		xX = Table[
			Xtransspatial[xS[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]],
			{i,1,xnj}];
		xXinv = Table[
			Xtransspatialinv[xSinv[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]],
			{i,1,xnj}];
			
		(* add the floating base in the zero element *)
		xX[[0]]    = Xtransspatial[xS[[0]],base[[2,{5,6,7}]],{0,0,0}];
		xXinv[[0]] = Xtransspatialinv[xSinv[[0]],base[[2,{5,6,7}]],{0,0,0}];
			  		
			  		
		(* build the spatial rotation vectors *)
		xs = Table[
			xax[[i]],
			{i,1,xnj}];
			       		  		
		(* build the spatial inertia matrices *)
		xJ = Table[
			InertiaSpatialSpec[xm[[i]],xmcm[[i]],xinert[[i]]],
			{i,1,xnj}];
			
		(* the base inertia can be copied from the base variable *)
		xJ[[0]] = base[[1]];
			       		  		
		(* build the velocity vectors *)
		xv = {}; 
		oxv = {}; 
		
		xvbase      = Vel2Spatial[base[[2,{5,6,7}]]*0,base[[3,{5,6,7}]],base[[3,{8,9,10}]]];
		xX0spec = xX[[0]];
		xX0spec[[{4,5,6},{1,2,3}]]=0; (* this eliminates the translation part *)
		oxv[[0]]    = xX0spec.xvbase;
		xv[[0]]     = MakeSimplerVector[oxv[[0]],ToString[StringForm["v````",0,"[[``]]"]],0];
		
		Do[
			{oxv=Append[oxv,Vispatial[xv[[xpred[[i]]]],xX[[i]],xs[[i]],xj[[i,2]]]] /. rep;
			xv=Append[xv,MakeSimplerVector[oxv[[i]],
			ToString[StringForm["v````",xID[[i]],"[[``]]"]],0]]},
			{i,1,xnj}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* velocity vectors */\n"];
		WriteOutput[filefun,Zt[xv],Zt[oxv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xv]],"[6+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
		
		
		(* build misc variable c *)
		oxc = Table[CMisc[xv[[i]],xs[[i]],xj[[i,2]]],{i,1,xnj}];
		xc  = Table[
			MakeSimplerVector[oxc[[i]],ToString[StringForm["c````",xID[[i]],"[[``]]"]],0],
			{i,1,xnj}];
		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* c-misc vectors */\n"];
		WriteOutput[filefun,xc,oxc];
		WriteDeclaration[filedec,"double  ",Flatten[xc],"[6+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
		
		
		(* build misc variables pv *)
		oxpv = Table[PVMisc[xv[[i]],xJ[[i]],xSG[[i]],xuext[[i]],gravity] /. rep,{i,1,xnj}];
		xpv  = Table[
			MakeSimplerVector[oxpv[[i]],ToString[StringForm["pv````",xID[[i]],"[[``]]"]],0],
			{i,1,xnj}];
		(* need the zero element of pv *)
		xuext[[0]]=base[[5]];
		oxpv[[0]] = PVMisc[xv[[0]],xJ[[0]],xSG[[0]],xuext[[0]],gravity] /. rep;
		xpv[[0]]  = MakeSimplerVector[oxpv[[0]],ToString[StringForm["pv````",0,"[[``]]"]],0];
		
		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* pv-misc vectors */\n"];
		WriteOutput[filefun,Zt[xpv],Zt[oxpv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xpv]],"[6+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
					  		  

		(* build the articulated body inertias *)
		xJA  = FillMatrix[{xnj,6,6},0];   (* need initialization since Do-loop runs backward *)
		oxJA = xJ;                        (* all xJA have xJ as additive element *)
		xh   = FillMatrix[{xnj,6},0];     (* the following are two misc variables *)
		oxh  = FillMatrix[{xnj,6},0];
		xd   = FillMatrix[{xnj},0];
		oxd  = FillMatrix[{xnj},0];
		xT1  = FillMatrix[{xnj,6,6},0];
		oxT1 = FillMatrix[{xnj,6,6},0];
		xT   = FillMatrix[{xnj,6,6},0];
		oxT  = FillMatrix[{xnj,6,6},0];
				
		(* note: I need to write the results to file inside the loop, since every
		   stage depends on multiple variables, i.e., JA,h,d,T. Thus dumping the
		   JA computations, then the h computations, etc., would violated the
		   recursion laws *)
		
		WriteString[filefun,"/* articulated body inertias and misc variables */\n"];
		

		xID[[0]] = 0; (* this fudge is needed below *)
		Do[
		{
			countfun = countfun+1;
			WriteFunctionStart[filefun,temp,countfun];
			
			(* at the i-th step, xJA is computed completely -- thus calculated the
			   misc variables at this step and add component to predecessor *)
			   
			xJA[[i]]  = MakeSimplerMatrix[oxJA[[i]],
				ToString[StringForm["JA````",xID[[i]],"[[``,``]]"]],0];
				
			If[ i==0,Null,
				oxh[[i]] = HMisc[xJA[[i]],xs[[i]]];
				xh[[i]]  = MakeSimplerVector[oxh[[i]],ToString[StringForm["h````",xID[[i]],"[[``]]"]],0];
		
				oxd[[i]] = DMisc[xs[[i]],xh[[i]]]+0.0000000001; (* for numerical stability *)
				xd[[i]]  = MakeSimplerVectorCoef[oxd[[i]],ToString[StringForm["d``","[[``]]"]],xID[[i]],0];
			
				oxT1[[i]] = TMisc1[xJA[[i]],xh[[i]],xd[[i]]] /. rep;
				xT1[[i]]  = MakeSimplerMatrix[oxT1[[i]],
					ToString[StringForm["T1``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0];
			
				oxT[[i]] = TMisc[xXinv[[i]],xX[[i]],xT1[[i]]] /. rep;
				xT[[i]]  = MakeSimplerMatrix[oxT[[i]],
					ToString[StringForm["T``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0];
			
				oxJA[[xpred[[i]]]] = oxJA[[xpred[[i]]]] + xT[[i]];
			];
			
			WriteOutput[filefun,xJA[[i]],oxJA[[i]]];
			If[i==0,Null, {
				WriteOutput[filefun,xh[[i]],oxh[[i]]];
				WriteOutput[filefun,{xd[[i]]},{oxd[[i]]}];
				WriteString[filefun,"/* make sure that d values are not numerically problematic  */\n"];
				WriteString[filefun,ToString[StringForm["if (fabs(d[``]) < 1.e-6) \n",i]]];
				WriteString[filefun,ToString[StringForm["  d[``]=macro_sign(d[``])*1.e-6;\n",i,i]]];
				WriteOutput[filefun,xT1[[i]],oxT1[[i]]];
				WriteOutput[filefun,xT[[i]],oxT[[i]]];}
			];

			WriteFunctionEnd[filefun,temp,countfun];
			WriteFunctionExe[file,temp,countfun];			  		   
			WriteFunctionDec[filedec,temp,countfun];			  		   
		},
		{i,xnj,0,-1}];

		WriteDeclaration[filedec,"double  ",Flatten[Zt[xJA]],"[6+1][6+1]"];
		WriteDeclaration[filedec,"double  ",Flatten[xh],"[6+1]"];
		WriteString[filedec,ToString[StringForm["double  d[``+1];\n\n",xnj]]];
		WriteDeclaration[filedec,"double  ",Flatten[xT1],"[6+1][6+1]"];
		WriteDeclaration[filedec,"double  ",Flatten[xT],"[6+1][6+1]"];
			  		   

		(* build the bias forces *)
		xp   = FillMatrix[{xnj,6},0];   (* need initialization since Do-loop runs backward *)
		oxp  = xpv;                     (* all bias forces have xpv as additive term *)
		xpmm = FillMatrix[{xnj,6},0];
		oxpmm= FillMatrix[{xnj,6},0];
		xpm  = FillMatrix[{xnj,6},0];
		oxpm = FillMatrix[{xnj,6},0];
		xu   = FillMatrix[{xnj},0];
		oxu  = FillMatrix[{xnj},0];
		
		(* note: I need to write the results to file inside the loop, since every
		   stage depends on multiple variables, i.e., xp and xu. Thus dumping the
		   xp computations, then the xu computations would violated the
		   recursion laws *)
		
		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* bias forces */\n"];

		Do[
		{

			(* at the i-th step, xp is computed completely -- thus calculate the
			   misc variables at this step and add component to predecessor *)
			   
			xp[[i]]  = MakeSimplerVector[oxp[[i]],
				ToString[StringForm["p````",xID[[i]],"[[``]]"]],0];
			   
			If[i==0,Null,
				oxu[[i]] = UMisc[xj[[i,4]],xh[[i]],xc[[i]],xs[[i]],xp[[i]]];
				xu[[i]]  = MakeSimplerVectorCoef[oxu[[i]],ToString[StringForm["u``","[[``]]"]],xID[[i]],0];
			
				oxpmm[[i]] = PMisc1[xp[[i]],xJA[[i]],xc[[i]],xu[[i]],xh[[i]],xd[[i]]] /. rep;
				xpmm[[i]]  = MakeSimplerVector[oxpmm[[i]],
					ToString[StringForm["pmm````",xID[[i]],"[[``]]"]],0];
				
				oxpm[[i]] = PMisc[xXinv[[i]],xpmm[[i]]] /. rep;
				xpm[[i]]  = MakeSimplerVector[oxpm[[i]],
					ToString[StringForm["pm````",xID[[i]],"[[``]]"]],0];
				
				oxp[[xpred[[i]]]] = oxp[[xpred[[i]]]] + xpm[[i]];
			];
			
			WriteOutput[filefun,xp[[i]],oxp[[i]]];
			If[i==0,Null,
				WriteOutput[filefun,{xu[[i]]},{oxu[[i]]}];
				WriteOutput[filefun,xpmm[[i]],oxpmm[[i]]];
				WriteOutput[filefun,xpm[[i]],oxpm[[i]]];
			];

		},
		{i,xnj,0,-1}];

		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];		
			  		   
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xp]],"[6+1]"];
		WriteDeclaration[filedec,"double  ",Flatten[xpmm],"[6+1]"];
		WriteDeclaration[filedec,"double  ",Flatten[xpm],"[6+1]"];
		WriteString[filedec,ToString[StringForm["double  u[``+1];\n\n",xnj]]];
			  		   
			  		   
		(* build the acceleration vectors and final joint accelerations *)
		xa = {}; 
		oxa = {};
		oxqdd = {};

		WriteString[file,"/* acceleration vectors and joint accelerations */\n"];
		
		(* for a floating base, the base acceleration needs special treatment, i.e.,
		   a matrix inversion *)
		If[base[[6]]==0,xa[[0]]={0,0,0,0,0,0},{
		(* solve for the accelerations *) 
		WriteString[file,"/* now solve for base accelerations: -xp[0] = JA[0]*add[0]  */\n"];
		WriteString[file,"/* Note: xp and JA are calculated above */\n"];
		WriteString[file,"MY_MATRIX(JAmat,1,6,1,6);\n"];
		WriteString[file,"for (i=1; i<=6; ++i)\n"];
		WriteString[file,"   for (j=1; j<=6; ++j) \n"];
		WriteString[file,"      JAmat[i][j]=JA0[i][j];\n"];
		WriteString[file,"my_inv_ludcmp_solve(JAmat,p0,6,a0);\n\n"];
		WriteString[file,"if (freeze_base) for (i=1; i<=6; ++i) a0[i]=0.0;\n\n"];
		WriteString[file,"\n\n"];

		oxa[[0]] = Table[nonsense,{i,1,6}];
		xa[[0]]  = MakeSimplerVector[oxa[[0]],ToString[StringForm["-a0``","[[``]]"]],1];
		
		(* xa is in base coordinates, not world coordinates. First, it needs to be rotated
		   into the world coordinates, and second, the orientational acceleration needs to
		   be converted into a quaternion acceleration. *)
		   
		xabase   = xXinv[[0]].xa[[0]];

		(* this step gives me the right translatory accelerations *)
		taux = Spatial2Acc[xabase,base[[2,{5,6,7}]],base[[3,{5,6,7}]],xvbase[[{1,2,3}]]];
		
		(* the quanternion acceleration needs to be added separately *)
		xafinal = Flatten[{AngAcc2Quat[base[[2,{1,2,3,4}]],base[[3,{1,2,3,4}]],xvbase[[{1,2,3}]],xabase[[{1,2,3}]]],taux[[2]],taux[[1]]}];
		
		WriteOutput[file,base[[4]],xafinal];}

		];

		Do[
		{
			oxqdd = Append[oxqdd,Thdd[xu[[i]],xh[[i]],xX[[i]],xa[[xpred[[i]]]],xd[[i]]]] /. rep;
			oxa=Append[oxa,AispatialAB[xa[[xpred[[i]]]],xX[[i]],xc[[i]],xs[[i]],xj[[i,3]]]] /. rep;
			xa=Append[xa,MakeSimplerVector[oxa[[i]],
					ToString[StringForm["a````",xID[[i]],"[[``]]"]],0]];

			WriteOutput[file,{xj[[i,3]]},{oxqdd[[i]]}];
			WriteOutput[file,xa[[i]],oxa[[i]]];
		},
		{i,1,xnj}];

		WriteDeclaration[filedec,"double  ",Flatten[Ot[-xa[[0]]]],"[6+1]"]; (* sign fix -- see above *)
		WriteDeclaration[filedec,"double  ",Flatten[Ot[xa]],"[6+1]"];
			  		   			  		   
			  		   			  		
		Close[file];
		Close[filedec];
		Close[filefun];


]

(************************* Inverse Dynamics Articulate Body  **********************)
(************************* Inverse Dynamics Articulate Body  **********************)
(************************* Inverse Dynamics Articulate Body  **********************)

InvDynArt[infile_,outfile_,gravity_]:=

	(* now we start calculating the inverse dynamics: note that this function is quick
	   adaptation from the Forward Dynamics Articulate Body methods.  *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,gravity,outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;
  					  		
		(* the output file *)
		temp=StringJoin[outfile,"_InvDynArt"];
		file    = StringJoin[temp,"_math.h"];
		filedec = StringJoin[temp,"_declare.h"];
		filefun = StringJoin[temp,"_functions.h"];
		OpenWrite[file];
		OpenWrite[filedec];
		OpenWrite[filefun];
			  		
		(* fix the numerics *)
		rep = {1.->1, -1.-> -1, 0.->0, 2.->2, 3.->3};
		
		(* a counter for the subprograms that we generate *)
		countfun=0;
			  		
		(* Note: ox... variables are output for C -- they are matched by
			a x... variable which contains the symbol which should be
			assigned, i.e, x... = ox... will be a proper C assignment *)
			  		
		(* precompute the sin and cos for each joint *)
		oxjsincos = Table[Flatten[{Sin[xj[[i,1]]],Cos[xj[[i,1]]]}(1-xisprismatic[[i]])],{i,1,xnj}];
		xjsincos = Table[MakeSinCos[xj[[i,1]]],{i,1,xnj}];
		WriteString[file,"/* sine and cosine precomputation */\n"];
		WriteOutput[file,xjsincos,oxjsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xjsincos],""];

			  		
		(* precompute the sin and cos for the rotation matrices *)
		oxrotsincos = 
			Table[Flatten[N[{Sin[xrot[[i,j]]],Cos[xrot[[i,j]]]}]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}] /. rep;
		xrotsincos = Table[MakeSinCosRot[xrot[[i,j]]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}] /. rep;

		WriteString[file,"/* rotation matrix sine and cosine precomputation */\n"];
		WriteOutput[file,xrotsincos,oxrotsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xrotsincos],""];
									
			  		   
		(* calculate the 3D rotation matrices *)
		oxS = Table[
			RotMatSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error1],
			{j,1,3}]].
			RotMatSpec[xrotsincos[[i]]],
			{i,1,xnj}];
		xS = Table[
			MakeSimplerMatrix[oxS[[i]],
				ToString[StringForm["S``````",xID[[i]],xpred[[i]],"[[``,``]]"]],0],
				{i,1,xnj}];
				
		(* rotation matrix for floating base *)
		oxS[[0]] = RotMatQuat[base[[2,{1,2,3,4}]]];
		xS[[0]] = MakeSimplerMatrix[oxS[[0]],
				ToString[StringForm["S``````",0,0,"[[``,``]]"]],0];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* rotation matrices */\n"];
		WriteOutput[filefun,Zt[xS],Zt[oxS]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xS]],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];		
			  		   
				
		(* need the inverse matrices as well *)
		oxSinv = Table[
			RotMatInvSpec[xrotsincos[[i]]].
			RotMatInvSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error2],
			{j,1,3}]],
			{i,1,xnj}] /. rep;
		xSinv = Table[
			MakeSimplerMatrix[oxSinv[[i]],
			ToString[StringForm["Si``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0],
			{i,1,xnj}];
		
		(* rotation matrix for floating base *)
		oxSinv[[0]] = RotMatQuatInv[base[[2,{1,2,3,4}]]] /. rep;
		xSinv[[0]] = MakeSimplerMatrix[oxSinv[[0]],
			ToString[StringForm["Si``````",0,0,"[[``,``]]"]],0];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* inverse rotation matrices */\n"];
		WriteOutput[filefun,Zt[xSinv],Zt[oxSinv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xSinv]],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
		

		(* for converting the external forces and gravity into local coordinates, the rotation
		   matrix from global to local coordinates of every joint is needed. This is
		   easily obtained from multiplying the S matrices with each other *)
		   
		xSG = {}; 
		oxSG = {}; 
		xSG[[0]] = xS[[0]]; (* note:changing the zero elements destroys the "list" identifier *)
		Do[
		{
			oxSG=Append[oxSG,xS[[i]].xSG[[xpred[[i]]]]] /. rep;
			xSG=Append[xSG,MakeSimplerMatrix[oxSG[[i]],
				ToString[StringForm["SG``0``",xID[[i]],"[[``,``]]"]],0]]
		},
		{i,1,xnj}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* rotation matrices from global to link coordinates */\n"];
		WriteOutput[filefun,xSG,oxSG];
		WriteDeclaration[filedec,"double  ",Flatten[Ot[xSG]],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];			  		   				
					  		   
			  		
		(* build the spatial rotation matrices *)
		xX = Table[
			Xtransspatial[xS[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]],
			{i,1,xnj}];
		xXinv = Table[
			Xtransspatialinv[xSinv[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]],
			{i,1,xnj}];
			
		(* add the floating base in the zero element *)
		xX[[0]]    = Xtransspatial[xS[[0]],base[[2,{5,6,7}]],{0,0,0}];
		xXinv[[0]] = Xtransspatialinv[xSinv[[0]],base[[2,{5,6,7}]],{0,0,0}];
			  		
			  		
		(* build the spatial rotation vectors *)
		xs = Table[
			xax[[i]],
			{i,1,xnj}];
			       		  		
		(* build the spatial inertia matrices *)
		xJ = Table[
			InertiaSpatialSpec[xm[[i]],xmcm[[i]],xinert[[i]]],
			{i,1,xnj}];
			
		(* the base inertia can be copied from the base variable *)
		xJ[[0]] = base[[1]];
			       		  		
		(* build the velocity vectors *)
		xv = {}; 
		oxv = {}; 
		
		xvbase      = Vel2Spatial[base[[2,{5,6,7}]]*0,base[[3,{5,6,7}]],base[[3,{8,9,10}]]];
		xX0spec = xX[[0]];
		xX0spec[[{4,5,6},{1,2,3}]]=0; (* this eliminates the translation part *)
		oxv[[0]]    = xX0spec.xvbase;
		xv[[0]]     = MakeSimplerVector[oxv[[0]],ToString[StringForm["v````",0,"[[``]]"]],0];
		
		Do[
			{oxv=Append[oxv,Vispatial[xv[[xpred[[i]]]],xX[[i]],xs[[i]],xj[[i,2]]]] /. rep;
			xv=Append[xv,MakeSimplerVector[oxv[[i]],
			ToString[StringForm["v````",xID[[i]],"[[``]]"]],0]]},
			{i,1,xnj}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* velocity vectors */\n"];
		WriteOutput[filefun,Zt[xv],Zt[oxv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xv]],"[6+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
		
		
		(* build misc variable c *)
		oxc = Table[CMisc[xv[[i]],xs[[i]],xj[[i,2]]] /. rep,{i,1,xnj}];
		xc  = Table[
			MakeSimplerVector[oxc[[i]],ToString[StringForm["c````",xID[[i]],"[[``]]"]],0],
			{i,1,xnj}];
		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* c-misc vectors */\n"];
		WriteOutput[filefun,xc,oxc];
		WriteDeclaration[filedec,"double  ",Flatten[xc],"[6+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
		
		
		(* build misc variables pv -- external forces are in link coordinates here *)
		oxpv = Table[PVMisc[xv[[i]],xJ[[i]],IdentityMatrix[3],xuext[[i]],xSG[[i]].gravity] /. rep,{i,1,xnj}];
		xpv  = Table[
			MakeSimplerVector[oxpv[[i]],ToString[StringForm["pv````",xID[[i]],"[[``]]"]],0],
			{i,1,xnj}];
		(* need the zero element of pv *)
		xuext[[0]]=base[[5]];
		oxpv[[0]] = PVMisc[xv[[0]],xJ[[0]],IdentityMatrix[3],xuext[[0]],xSG[[0]].gravity] /. rep;
		xpv[[0]]  = MakeSimplerVector[oxpv[[0]],ToString[StringForm["pv````",0,"[[``]]"]],0];
	

		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* pv-misc vectors */\n"];
		WriteOutput[filefun,Zt[xpv],Zt[oxpv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xpv]],"[6+1]"];
		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];
					  		  

		(* build the articulated body inertias *)
		xJA  = FillMatrix[{xnj,6,6},0];   (* need initialization since Do-loop runs backwards *)
		oxJA = xJ;                        (* all xJA have xJ as additive element *)
		xh   = FillMatrix[{xnj,6},0];     (* the following are two misc variables *)
		oxh  = FillMatrix[{xnj,6},0];
		xT1  = FillMatrix[{xnj,6,6},0];
		oxT1 = FillMatrix[{xnj,6,6},0];
		xT   = FillMatrix[{xnj,6,6},0];
		oxT  = FillMatrix[{xnj,6,6},0];
				
		(* note: I need to write the results to file inside the loop, since every
		   stage depends on multiple variables, i.e., JA,h,d,T. Thus dumping the
		   JA computations, then the h computations, etc., would violate the
		   recursion laws *)
		
		WriteString[filefun,"/* articulated body inertias and misc variables */\n"];
		

		xID[[0]] = 0; (* this fudge is needed below *)
		Do[
		{
			countfun = countfun+1;
			WriteFunctionStart[filefun,temp,countfun];
			
			(* at the i-th step, xJA is computed completely -- thus calculated the
			   misc variables at this step and add component to predecessor *)
			   
			xJA[[i]]  = MakeSimplerMatrix[oxJA[[i]],
				ToString[StringForm["JA````",xID[[i]],"[[``,``]]"]],0];
							
			If[ i==0,Null,
			
				oxh[[i]] = HMisc[xJA[[i]],xs[[i]]];
				xh[[i]]  = MakeSimplerVector[oxh[[i]],ToString[StringForm["h````",xID[[i]],"[[``]]"]],0];
		
				(* the zero multiplier cancels the terms that are not needed for inverse dynamics,
				   and xd was replaced by 1 *)
				oxT1[[i]] = TMisc1[xJA[[i]],xh[[i]]*0,1] /. rep;
				xT1[[i]]  = MakeSimplerMatrix[oxT1[[i]],
				ToString[StringForm["T1``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0];
			
				oxT[[i]] = TMisc[xXinv[[i]],xX[[i]],xT1[[i]]] /. rep;
				xT[[i]]  = MakeSimplerMatrix[oxT[[i]],
				ToString[StringForm["T``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0];
			
				oxJA[[xpred[[i]]]] = oxJA[[xpred[[i]]]] + xT[[i]];
			];
			
			WriteOutput[filefun,xJA[[i]],oxJA[[i]]];
			
			If[i==0,Null,
				WriteOutput[filefun,xh[[i]],oxh[[i]]];
				WriteOutput[filefun,xT1[[i]],oxT1[[i]]];
				WriteOutput[filefun,xT[[i]],oxT[[i]]];
			];

			WriteFunctionEnd[filefun,temp,countfun];
			WriteFunctionExe[file,temp,countfun];			  		   
			WriteFunctionDec[filedec,temp,countfun];			  		   
		},
		{i,xnj,0,-1}];

		WriteDeclaration[filedec,"double  ",Flatten[Zt[xJA]],"[6+1][6+1]"];
		WriteDeclaration[filedec,"double  ",Flatten[xh],"[6+1]"];
		WriteDeclaration[filedec,"double  ",Flatten[xT1],"[6+1][6+1]"];
		WriteDeclaration[filedec,"double  ",Flatten[xT],"[6+1][6+1]"];
			  		   

		(* build the bias forces *)
		xp   = FillMatrix[{xnj,6},0];   (* need initialization since Do-loop runs backward *)
		oxp  = xpv;                     (* all bias forces have xpv as additive term *)
		xpmm = FillMatrix[{xnj,6},0];
		oxpmm= FillMatrix[{xnj,6},0];
		xpm  = FillMatrix[{xnj,6},0];
		oxpm = FillMatrix[{xnj,6},0];
		
		(* note: I need to write the results to file inside the loop, since every
		   stage depends on multiple variables, i.e., xp and xu. Thus dumping the
		   xp computations, then the xu computations would violated the
		   recursion laws *)
		
		countfun = countfun+1;
		WriteFunctionStart[filefun,temp,countfun];
		WriteString[filefun,"/* bias forces */\n"];

		Do[
		{

			(* at the i-th step, xp is computed completely -- thus calculate the
			   misc variables at this step and add component to predecessor *)
			   
			xp[[i]]  = MakeSimplerVector[oxp[[i]],
				ToString[StringForm["p````",xID[[i]],"[[``]]"]],0];
			   
			If[i==0,Null,
			
				(* an "abuse" of the PMisc1 function allows to reduce the computations to what inv.dyn needs *)
				oxpmm[[i]] = PMisc1[xp[[i]],xJA[[i]],xc[[i]],xj[[i,3]],xh[[i]],1] /. rep;
				xpmm[[i]]  = MakeSimplerVector[oxpmm[[i]],
					ToString[StringForm["pmm````",xID[[i]],"[[``]]"]],0];
				
				oxpm[[i]] = PMisc[xXinv[[i]],xpmm[[i]]] /. rep;
				xpm[[i]]  = MakeSimplerVector[oxpm[[i]],
					ToString[StringForm["pm````",xID[[i]],"[[``]]"]],0];
				
				oxp[[xpred[[i]]]] = oxp[[xpred[[i]]]] + xpm[[i]];
				
			];
			
			WriteOutput[filefun,xp[[i]],oxp[[i]]];
			If[i==0,Null,
				WriteOutput[filefun,xpmm[[i]],oxpmm[[i]]];
				WriteOutput[filefun,xpm[[i]],oxpm[[i]]];
			];

		},
		{i,xnj,0,-1}];

		WriteFunctionEnd[filefun,temp,countfun];
		WriteFunctionExe[file,temp,countfun];			  		   
		WriteFunctionDec[filedec,temp,countfun];		
			  		   
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xp]],"[6+1]"];
		WriteDeclaration[filedec,"double  ",Flatten[xpmm],"[6+1]"];
		WriteDeclaration[filedec,"double  ",Flatten[xpm],"[6+1]"];
			  		   
		(* build the acceleration vectors and final joint accelerations/forces *)
		xa = {}; 
		oxa = {};

		WriteString[file,"/* acceleration vectors, base acceleration, and joint torques */\n"];
		
		(* for a floating base, the base acceleration needs special treatment, i.e.,
		   a matrix inversion *)
		
		If[base[[6]]==0,xa[[0]]={0,0,0,0,0,0},{
		(* solve for the accelerations *) 
		WriteString[file,"/* now solve for base accelerations: -xp[0] = JA[0]*add[0]  */\n"];
		WriteString[file,"/* Note: xp and JA are calculated above */\n"];
		WriteString[file,"MY_MATRIX(JAmat,1,6,1,6);\n"];
		WriteString[file,"for (i=1; i<=6; ++i)\n"];
		WriteString[file,"   for (j=1; j<=6; ++j) \n"];
		WriteString[file,"      JAmat[i][j]=JA0[i][j];\n"];
		WriteString[file,"my_inv_ludcmp_solve(JAmat,p0,6,a0);\n\n"];
		WriteString[file,"if (freeze_base) for (i=1; i<=6; ++i) a0[i]=0.0;\n\n"];
		WriteString[file,"\n\n"];

		oxa[[0]] = Table[nonsense,{i,1,6}];
		xa[[0]]  = MakeSimplerVector[oxa[[0]],ToString[StringForm["-a0``","[[``]]"]],1];
		xabase   = xXinv[[0]].xa[[0]];
		
		(* this step gives me the right translatory accelerations *)
		taux = Spatial2Acc[xabase,base[[2,{5,6,7}]],base[[3,{5,6,7}]],xvbase[[{1,2,3}]]];
		
		(* the quanternion acceleration needs to be added separately *)
		xafinal = Flatten[{AngAcc2Quat[base[[2,{1,2,3,4}]],base[[3,{1,2,3,4}]],xvbase[[{1,2,3}]],xabase[[{1,2,3}]]],taux[[2]],taux[[1]]}];
		
		WriteOutput[file,base[[4]],xafinal];}

		];

		(* and finally the inverse dynamics joint torques *)
		xq = Table[xj[[i,4]],{i,1,xnj}];
		(* if we use CLMC structures, the u must become uff for inv.dyn *)
		xq=ToExpression[StringReplace[ToString[xq,OutputForm] ,"$$u"->"$$uff"]];

		Do[
		{
			oxa=Append[oxa,AispatialAB[xa[[xpred[[i]]]],xX[[i]],xc[[i]],xs[[i]],xj[[i,3]]]] /. rep;
			xa=Append[xa,MakeSimplerVector[oxa[[i]],
					ToString[StringForm["a````",xID[[i]],"[[``]]"]],0]];
			(* xq is new, and again, the Qjoint function can be fudged to compute the torques *)
			oxq[[i]] = Qjoint[xs[[i]],(xJA[[i]].xa[[i]]+xp[[i]]) ] /. rep;

			WriteOutput[file,xa[[i]],oxa[[i]]];

		},
		{i,1,xnj}];
		
		WriteString[file,"/* inverse dynamics torques */\n"];
		WriteOutput[file,xq,oxq];

		WriteDeclaration[filedec,"double  ",Flatten[Ot[-xa[[0]]]],"[6+1]"]; (* sign fix -- see above *)
		WriteDeclaration[filedec,"double  ",Flatten[Ot[xa]],"[6+1]"];
					  		   			  		
		Close[file];
		Close[filedec];
		Close[filefun];


]

(************************* Forward Dynamics Comp.Inertia  **********************)
(************************* Forward Dynamics Comp.Inertia  **********************)
(************************* Forward Dynamics Comp.Inertia  **********************)

ForDynComp[infile_,outfile_,gravity_]:=

	(* now we start calculating the forward dynamics *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,gravity,outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;

		(* zero the accelerations in xl and calculate the inverse
				dynamics for this special case *)
				
		oxc = Table[nonsense,{i,1,xnj+6}];
		xc  = MakeSimplerVector[oxc,ToString[StringForm["c``","[[``]]"]],0];

  	  Do[
  			(xl[[12,i,3]]=0;xl[[12,i,5]]=0;xl[[12,i,4]]=xc[[i]]),
  			{i,1,xnj}];

		xl[[14,4]] = xl[[14,4]]*0;
		xl[[14,5]] = xl[[14,5]]*0;
		
		temp=StringJoin[outfile,"_ForDynComp"];
		InvDynSpec[xl,temp,gravity];
		
		(* a counter for the subprograms that we generate *)
		countfun=10;

		(* re-open the output files *)
		file    = StringJoin[temp,"_math.h"];
		filedec = StringJoin[temp,"_declare.h"];
		filefun = StringJoin[temp,"_functions.h"];
		OpenAppend[file];
		OpenAppend[filedec];
		OpenAppend[filefun];
		WriteDeclaration[filedec,"double  ",Flatten[xc],ToString[StringForm["[``+1]",Length[xc]]]];
		
		(* in order to derive the inertia matrix in conventional (non-spatial) terms, we need a special
		   inverse coordinate transformation from base to world coordinates, that does not shift the spatial
		   vectors to the origin of the world, but rather keeps the orgin at the base coordinates *)
		xXinv0spec = xXinv[[0]];
		xXinv0spec[[{4,5,6},{1,2,3}]]=0; (* this eliminates the translation part *)
		xX0spec = xX[[0]];
		xX0spec[[{4,5,6},{1,2,3}]]=0; (* this eliminates the translation part *)
			  					  		
		(* from here on, special operations for the forward dynamics are required *)
		(* the composite inertia matrices *)
		xJc = FillMatrix[{xnj,6,6},0]; (* need xJc initialized since Do-loop runs backward and indexes into it *)
		oxJc = xJ; (* all xJc have xJ as additive component *)
		
		Do[
			{Do[ oxJc[[i]] = oxJc[[i]] + 
				InertiaSpatialComp[xXinv[[xsuc[[i,j]]]],xJc[[xsuc[[i,j]]]],xX[[xsuc[[i,j]]]]] /. rep,
			{j,1,Length[xsuc[[i]]]}];
			If[i==0,
			xJc[[i]]=MakeSimplerMatrix[oxJc[[i]],ToString[StringForm["Jcc````",xID[[i]],"[[``,``]]"]],1],
			xJc[[i]]=MakeSimplerMatrix[oxJc[[i]],ToString[StringForm["Jc````",xID[[i]],"[[``,``]]"]],1]];
			countfun = countfun+1;
			WriteFunctionStart[filefun,outfile,countfun];
			WriteString[filefun,"/* composite inertia matrices */\n"];
			WriteOutput[filefun,xJc[[i]],oxJc[[i]]];
			WriteFunctionEnd[filefun,outfile,countfun];
			WriteFunctionExe[file,outfile,countfun];			  		   
			WriteFunctionDec[filedec,outfile,countfun];
			
			(* put the base Jc into world coordinates, but without translation *)
			If[i==0,
				{
					WriteString[filedec,ToString[StringForm["double  Jcc0[6+1][6+1];\n\n"]]];
					oxJc[[0]]=InertiaSpatialComp[xXinv0spec,xJc[[0]],xX0spec] /. rep;
					xJc[[i]]=MakeSimplerMatrix[oxJc[[i]],ToString[StringForm["Jc````",xID[[i]],"[[``,``]]"]],1];
					countfun = countfun+1;
					WriteFunctionStart[filefun,outfile,countfun];
					WriteString[filefun,"/* composite inertia matrices */\n"];
					WriteOutput[filefun,xJc[[i]],oxJc[[i]]];
					WriteFunctionEnd[filefun,outfile,countfun];
					WriteFunctionExe[file,outfile,countfun];			  		   
					WriteFunctionDec[filedec,outfile,countfun]
				},
				Null];
					  		   
			},
			{i,xnj,0,-1}];

		WriteDeclaration[filedec,"double  ",Flatten[Zt[xJc]],"[6+1][6+1]"];

		(* the forces in local coordinates that generate a unit acceleration at each joint *)
		xfu = Table[xJc[[i]].xs[[i]] /. rep,{i,1,xnj}];

		(* transform the unit forces to all lower levels -- this also creates
		   the K matrix, i.e., the off-diagonal terms in the inertia matrix of the
		   DOFS plus base coordinates *)
		xK=FillMatrix[{xnj,6},0];
		oxK=xK;
		oxFu = Table[If[i==j,xfu[[i]],{0,0,0,0,0,0}],{i,1,xnj},{j,1,xnj}];
		xFu = oxFu;
		
		Do[(
			j = i;
			xFu[[i,i]]=MakeSimplerVector[oxFu[[i,i]],
				ToString[StringForm["Fu``````",xID[[i]],xID[[i]],"[[``]]"]],0];

			While[j != 0, 
				(If [xpred[[j]] == 0,
				(* express K in world coordinates that are centered at the base *)
				(oxK[[i]] =Ftransdown[Ftransdown[xFu[[i,j]],xXinv[[j]]],xXinv0spec] /. rep),
				(oxFu[[i,xpred[[j]]]]=Ftransdown[xFu[[i,j]],xXinv[[j]]] /. rep;
				  xFu[[i,xpred[[j]]]]=MakeSimplerVector[oxFu[[i,xpred[[j]]]],
					ToString[StringForm["Fu``````",xID[[i]],xID[[xpred[[j]]]],"[[``]]"]],0]
				)
				];
			j = xpred[[j]])
			];
		),
		{i,xnj,1,-1}];
		
		xK = MakeSimplerMatrix[oxK,ToString[StringForm["K``","[[``,``]]"]],1];

		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];

		WriteString[filefun,"/* unit force elements of inertia matrix */\n"];
		Do[WriteOutput[filefun,Reverse[xFu[[i]]],Reverse[oxFu[[i]]]],{i,xnj,1,-1}];
		WriteDeclaration[filedec,"double  ",Flatten[xFu],"[6+1]"];
		
		WriteString[filefun,"/* off-diagonal elements of full inertia matrix */\n"];
		WriteOutput[filefun,xK,oxK];
		WriteDeclaration[filedec,"double  ",Flatten[xK],ToString[StringForm["[``+1][``+1]",xnj,6]]];
		
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];	
				  		   

		(* and finally the inertia matrix *)
		oxH = Table[ Hijcomponent[xs[[j]],xFu[[i,j]]] /. rep,{i,1,xnj},{j,1,xnj}];
		(* for fake links that are only given for esthetic and endeffector reason,
		   the inertia matrix has only zero entries. The diagonal entry in H for 
		   these elements needs to be set to ONE such that the matrix inversion still
		   works *)
		For[i=1, i<=xnj, ++i, If[DummyLinkQ[xj[[i,3]]],oxH[[i,i]]=1,Null]];
		xH = MakeSimplerMatrix[oxH,ToString[StringForm["H``","[[``,``]]"]],1];

		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];
		WriteString[filefun,"/* elements of inertia matrix */\n"];
		WriteOutput[filefun,xH,oxH];
		WriteDeclaration[filedec,"double  ",Flatten[xH],ToString[StringForm["[``+1][``+1]",xnj,xnj]]];
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];			  		   

		(* we need u - c, i.e.., the real commands minus the zero-acc commands. For floating
		   base computations, we also need to add gravity and bias forces acting on the base,
		   which was computed in the xf[[0]] and xfext[[0]] variables using the inverse dynamics 
		   function above *) 
		   
		(* first, c needs a velocity cross product correction term such that we can compute 
		   in non-spatial quantities *)
		oxvcross = CrosstoMatrix[base[[3,{5,6,7}]]].xvbase[[{1,2,3}]];
		xvcross  = MakeSimplerVector[oxvcross,ToString[StringForm["vcross``","[[``]]"]],1];
		WriteOutput[file,xvcross,oxvcross];
		WriteString[filedec,ToString[StringForm["double  vcross[3+1];\n\n"]]];
		
		(* for computing xc, the external forces are subtracted such that xc is realy C+G;
		   Note that xf[[0]] does not contain external forces *)
		oxc = Table[xc[[i]]-xqext[[i]]+xK[[i,{1,2,3}]].xvcross,{i,1,xnj}];
		oxc = Flatten[Append[oxc,xXinv0spec.xf[[0]]+xJc[[0]].Flatten[{0,0,0,xvcross}] ]] /. rep;
		xc  = MakeSimplerVector[oxc,ToString[StringForm["c``","[[``]]"]],1];
		WriteOutput[file,xc,oxc];
		   
		(* now compute u-c term which is finally needed in matrix inversion *)
		(* oxuc = Table[xj[[i,4]]+xj[[i,5]]-xc[[i]],{i,1,xnj}]   do we need external torques? leftover .... *)
		oxuc = Table[-(xc[[i]]+xqext[[i]]),{i,1,xnj}];
		oxuc = Flatten[Append[oxuc,-(xc[[{xnj+1,xnj+2,xnj+3,xnj+4,xnj+5,xnj+6}]]+xXinv0spec.xfext[[0]])]] /. rep;
		For [i=1, i<=xnj, ++i, oxuc[[i]] = xj[[i,4]] + oxuc[[i]] ];
		

		xuc  = MakeSimplerVector[oxuc,ToString[StringForm["uc``","[[``]]"]],1];
		WriteOutput[file,xuc,oxuc];
		WriteDeclaration[filedec,"double  ",Flatten[xuc],ToString[StringForm["[``+1]",Length[xuc]]]];
		
		oxthdd = Table[nonsense,{i,1,xnj+6}];
		xthdd  = MakeSimplerVector[oxthdd,ToString[StringForm["thdd``","[[``]]"]],1];
		WriteDeclaration[filedec,"double  ",Flatten[xthdd],ToString[StringForm["[``+1]",Length[xthdd]]]];

		(* solve for the accelerations *) 
		WriteString[file,"/* now solve for accelerations: uc = H * thdd           */\n"];
		WriteString[file,"/* Note: uc and H are calculated above */\n"];
		
		(* sort the all coefficients of H into Hmat such that we have original joint numbering and
		   a compact matrix *)
		   
		For[i=1, i<=xnj, ++i,{
			For[j=i, j<=xnj, ++j,{
				If [ DummyLinkQ[xj[[i,3]]] || DummyLinkQ[xj[[j,3]]], Null,(
				   If [ xIDorg[[i]] == xIDorg[[j]],
				   WriteString[file,ToString[StringForm["Hmat[``][``] = H[``][``];\n",xIDorg[[i]],xIDorg[[j]],j,i ]]],
				   WriteString[file,ToString[StringForm["Hmat[``][``] = Hmat[``][``] = H[``][``];\n",xIDorg[[i]],xIDorg[[j]],xIDorg[[j]],xIDorg[[i]],j,i ]]]
				];
				),
				(
				   If [ xIDorg[[i]] == xIDorg[[j]],
				   WriteString[file,ToString[StringForm["Hmat[``][``] = H[``][``];\n",xIDorg[[i]],xIDorg[[j]],j,i ]]],
				   WriteString[file,ToString[StringForm["Hmat[``][``] = Hmat[``][``] = H[``][``];\n",xIDorg[[i]],xIDorg[[j]],xIDorg[[j]],xIDorg[[i]],j,i ]]]
				];
				)
				]
			}]
		}];
		 
		(* also write out the correctly arranged c and uc vectors *)
		nrj = 0;
		For[i=1, i<=xnj, ++i,{
			If [ DummyLinkQ[xj[[i,3]]],Null,(
				nrj = nrj+1;
				WriteString[file,ToString[StringForm["cvec[``]  = c[``];\n",xIDorg[[i]],i ]]];
				WriteString[file,ToString[StringForm["ucvec[``] = uc[``];\n",xIDorg[[i]],i ]]];
			),
			(
				nrj = nrj+1;
				WriteString[file,ToString[StringForm["cvec[``]  = c[``];\n",xIDorg[[i]],i ]]];
				WriteString[file,ToString[StringForm["ucvec[``] = uc[``];\n",xIDorg[[i]],i ]]];
			)
			]
		}];

		WriteString[file," for (i=1; i<=6; ++i) \n"];
		WriteString[file,ToString[StringForm["      cvec[``+i]=c[``+i];\n",nrj,xnj]]];
		WriteString[file," for (i=1; i<=6; ++i) \n"];
		WriteString[file,ToString[StringForm["      ucvec[``+i]=uc[``+i];\n",nrj,xnj]]];
		
		
		(* if this is a floating base computation, the inertia matrix needs to include the
		   base -- note that some swapping of columns is needed to obtain a symmetric matrix.
		   In this form, the base acceleration is the conventional acceleration in the 
		   arrangement of rdd wd, i.e., opposite arrangement than spatial notation *)
		If[base[[6]]==0,{
			xa[[0]]={0,0,0,0,0,0};
			WriteString[file,ToString[StringForm["my_inv_ldlt(Hmat,ucvec,``,thdd);\n\n",nrj]]]
			},{
			WriteString[file," for (i=1; i<=6; ++i) \n"];
			WriteString[file,"   for (j=1; j<=3; ++j) \n"];
			WriteString[file,ToString[StringForm["      Hmat[``+i][``+j]=Jc0[i][j+3];\n",nrj,nrj]]];
			WriteString[file," for (i=1; i<=6; ++i) \n"];
			WriteString[file,"   for (j=4; j<=6; ++j) \n"];
			WriteString[file,ToString[StringForm["      Hmat[``+i][``+j]=Jc0[i][j-3];\n",nrj,nrj]]];
			
			For[i=1, i<=xnj, ++i,{
				For[j=1, j<=6, ++j,{
					If [ DummyLinkQ[xj[[i,3]]],Null,(
				   	If [ xIDorg[[i]] == j+nrj,
						WriteString[file,ToString[StringForm["Hmat[``][``] = K[``][``];\n",xIDorg[[i]],j+nrj,i,j]]],
						WriteString[file,ToString[StringForm["Hmat[``][``] = Hmat[``][``] = K[``][``];\n",xIDorg[[i]],j+nrj,j+nrj,xIDorg[[i]],i,j]]]
					   ];
					),
					(
				   	If [ xIDorg[[i]] == j+nrj,
						WriteString[file,ToString[StringForm["Hmat[``][``] = K[``][``];\n",xIDorg[[i]],j+nrj,i,j]]],
						WriteString[file,ToString[StringForm["Hmat[``][``] = Hmat[``][``] = K[``][``];\n",xIDorg[[i]],j+nrj,j+nrj,xIDorg[[i]],i,j]]]
					   ];
					)
					]
				}]
			}];
			
			
			WriteString[file,"if (freeze_base)\n"];
			WriteString[file,ToString[StringForm["  my_inv_ldlt(Hmat,ucvec,``,thdd);\n",nrj]]];
			WriteString[file,"else\n"];
			WriteString[file,ToString[StringForm["  my_inv_ldlt(Hmat,ucvec,``,thdd);\n\n",nrj+6]]];
			

			
			xa[[0]] = xthdd[[{nrj+1,nrj+2,nrj+3,nrj+4,nrj+5,nrj+6}]]; (* note that we have
				                                                         translatory acc. first,then angular acc. *)
			
			(* need to convert the angular acceleration to a quaternion acceleration; all accelerations are
			   in non-spatial world coordinate notation *)
			
			(* xa is in world coordinates. The orientational acceleration needs to
		  	be converted into a quaternion acceleration. Note that the quaternion acceleration
		   	requires ang.vel and ang.acc in BASE coordinates, not WORLD coordintates, which
		   	requires insertion of a coordinate rotation *)

			(* the quanternion acceleration *)
			xafinal = Flatten[{AngAcc2Quat[base[[2,{1,2,3,4}]],base[[3,{1,2,3,4}]],xvbase[[{1,2,3}]],
			                   xa[[0,{4,5,6}]]],xa[[0,{1,2,3}]],xa[[0,{4,5,6}]]}];
		
			WriteOutput[file,base[[4]],xafinal];
			
			WriteString[file,"if (freeze_base) {\n"];
			WriteOutput[file,base[[4]],xafinal*0 /. rep];
			WriteString[file,"}\n"];
		}];
		   
		(* and assign to final variables *) 
		oxtemp=xthdd;
		xtemp=Table[xj[[i,3]],{i,1,xnj}];
		For[i=1, i<=xnj,++i, 
			If [ DummyLinkQ[xj[[i,3]]],Null,xtemp[[xIDorg[[i]]]] = xj[[i,3]],xtemp[[xIDorg[[i]]]] = xj[[i,3]]]
		];
		(* clip the output vector to the right size *)
		oxtemp = Table[oxtemp[[i]],{i,1,nrj}];
		xtemp  = Table[xtemp[[i]],{i,1,nrj}];
		WriteOutput[file,xtemp,oxtemp];

		Close[file];
		Close[filedec];
		Close[filefun];
	]

(************************* Inertia Matrix  **********************************)
(************************* Inertia Matrix  **********************************)
(************************* Inertia Matrix  **********************************)

InertiaMatrix[infile_,outfile_]:=

	(* now we start calculating the inertia matrix *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,{0,0,0},outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;

		temp=StringJoin[outfile,"_InertiaMatrix"];

		(* a counter for the subprograms that we generate *)
		countfun=10;

		(* open the output files *)
		file    = StringJoin[temp,"_math.h"];
		filedec = StringJoin[temp,"_declare.h"];
		filefun = StringJoin[temp,"_functions.h"];
		Open[file];
		Open[filedec];
		Open[filefun];
		
		(* fix the numerics *)
		rep = {1.->1, -1.-> -1, 0.->0, 2.->2, 3.->3};
			  					  		
		(* precompute the sin and cos for each joint *)
		oxjsincos = Table[Flatten[{Sin[xj[[i,1]]],Cos[xj[[i,1]]]}(1-xisprismatic[[i]])],{i,1,xnj}];
		xjsincos = Table[MakeSinCos[xj[[i,1]]],{i,1,xnj}];
		WriteString[file,"/* sine and cosine precomputation */\n"];
		WriteOutput[file,xjsincos,oxjsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xjsincos],""];

			  		
		(* precompute the sin and cos for the rotation matrices *)
		oxrotsincos = 
			Table[Flatten[N[{Sin[xrot[[i,j]]],Cos[xrot[[i,j]]]}]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}] /. rep;
		xrotsincos = Table[MakeSinCosRot[xrot[[i,j]]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}] /. rep;
		WriteString[file,"/* rotation matrix sine and cosine precomputation */\n"];
		WriteOutput[file,xrotsincos,oxrotsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xrotsincos],""];
			  		   
		(* calculate the 3D rotation matrices *)
		oxS = Table[
			RotMatSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error1],
			{j,1,3}]].
			RotMatSpec[xrotsincos[[i]]],
			{i,1,xnj}];
		xS = Table[
			MakeSimplerMatrix[oxS[[i]],
				ToString[StringForm["S``````",xID[[i]],xpred[[i]],"[[``,``]]"]],0],
				{i,1,xnj}];
		
		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];
		WriteString[filefun,"/* rotation matrices */\n"];
		WriteOutput[filefun,xS,oxS];
		WriteDeclaration[filedec,"double  ",Flatten[xS],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];			  		   
				
		(* need the inverse matrices as well *)
		oxSinv = Table[
			RotMatInvSpec[xrotsincos[[i]]].
			RotMatInvSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error2],
			{j,1,3}]],
			{i,1,xnj}] /. rep;
		xSinv = Table[
			MakeSimplerMatrix[oxSinv[[i]],
			ToString[StringForm["Si``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0],
			{i,1,xnj}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];
		WriteString[filefun,"/* inverse rotation matrices */\n"];
		WriteOutput[filefun,xSinv,oxSinv];
		WriteDeclaration[filedec,"double  ",Flatten[xSinv],"[3+1][3+1]"];
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];			  		   
			  		
		(* build the spatial rotation matrices *)
		xX = Table[
			Xtransspatial[xS[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]],
			{i,1,xnj}];
		xXinv = Table[
			Xtransspatialinv[xSinv[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]],
			{i,1,xnj}];
			  		
		(* build the spatial rotation vectors *)
		xs = Table[
			xax[[i]],
			{i,1,xnj}];
			       		  		
		(* build the spatial inertia matrices *)
		xJ = Table[
			InertiaSpatialSpec[xm[[i]],xmcm[[i]],xinert[[i]]],
			{i,1,xnj}];
			       		  		
		(* the composite inertia matrices *)
		xJc = xJ*0; (* need xJc initialized since Do-loop runs backward and indexes into it *)
		oxJc = xJ; (* all xJc have xJ as additive component *)
		Do[
			{Do[ oxJc[[i]] = oxJc[[i]] + 
				InertiaSpatialComp[xXinv[[xsuc[[i,j]]]],xJc[[xsuc[[i,j]]]],xX[[xsuc[[i,j]]]]] /. rep,
			{j,1,Length[xsuc[[i]]]}];
			xJc[[i]]=MakeSimplerMatrix[oxJc[[i]],ToString[StringForm["Jc````",xID[[i]],"[[``,``]]"]],0]
			},
			{i,xnj,1,-1}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];
		WriteString[filefun,"/* composite inertia matrices */\n"];
		WriteOutput[filefun,Reverse[xJc],Reverse[oxJc]];
		WriteDeclaration[filedec,"double  ",Flatten[xJc],"[6+1][6+1]"];
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];			  		   

		(* the forces that generate a unit acceleration at each joint *)
		xfu = Table[xJc[[i]].xs[[i]] /. rep,{i,1,xnj}];

		(* transform the unit forces to all lower levels *)
		oxFu = Table[If[i==j,xfu[[i]],xfu[[1]]*0],{i,1,xnj},{j,1,xnj}];
		xFu = oxFu;
		Do[
			(If[i==j,(successor=i;predecessor=xpred[[i]]),Null];
			If[ predecessor==j, (oxFu[[i,j]]=Ftransdown[xFu[[i,successor]],xXinv[[successor]]] /. rep;
				successor=j;predecessor=xpred[[j]]),Null];
			xFu[[i,j]] = MakeSimplerVector[oxFu[[i,j]],ToString[StringForm["Fu``````",xID[[i]],xID[[j]],"[[``]]"]],0];
			),
			{i,xnj,1,-1},{j,i,1,-1}];

		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];
		WriteString[filefun,"/* unit force elements of inertia matrix */\n"];
		Do[WriteOutput[filefun,Reverse[xFu[[i]]],Reverse[oxFu[[i]]]],{i,xnj,1,-1}];
		WriteDeclaration[filedec,"double  ",Flatten[xFu],"[6+1]"];
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];			  		   
		
		(* and finally the inertia matrix *)
		oxH = Table[ Hijcomponent[xs[[j]],xFu[[i,j]]] /. rep,{i,1,xnj},{j,1,xnj}];
		(* for fake links that are only given for esthetic and endeffector reason,
		   the inertia matrix has only zero entries. The diagonal entry in H for 
		   these elements needs to be set to ONE such that the matrix inversion still
		   works *)
		For[i=1, i<=xnj, ++i, If[DummyLinkQ[xj[[i,3]]],oxH[[i,i]]=1,Null]];
		xH = MakeSimplerMatrix[oxH,ToString[StringForm["H``","[[``,``]]"]],1];
	
		countfun = countfun+1;
		WriteFunctionStart[filefun,outfile,countfun];
		WriteString[filefun,"/* elements of inertia matrix */\n"];
		WriteOutput[filefun,xH,oxH];
		WriteDeclaration[filedec,"double  ",Flatten[xH],ToString[StringForm["[``+1][``+1]",xnj,xnj]]];
		WriteFunctionEnd[filefun,outfile,countfun];
		WriteFunctionExe[file,outfile,countfun];			  		   
		WriteFunctionDec[filedec,outfile,countfun];			  		   

		(* assign the inertia matrix elements to the global matrix *) 
		WriteString[file,"/* final inertia matrix result (need to sort out dummy DOFS) */\n"];
		For[i=1, i<=xnj, ++i,{
			For[j=i, j<=xnj, ++j,{
				If [ DummyLinkQ[xj[[i,3]]] || DummyLinkQ[xj[[j,3]]],Null,
				WriteString[file,ToString[StringForm["Hmat[``][``] = Hmat[``][``] = H[``][``];\n",xIDorg[[i]],xIDorg[[j]],xIDorg[[j]],xIDorg[[i]],j,i ]]],
				WriteString[file,ToString[StringForm["Hmat[``][``] = Hmat[``][``] = H[``][``];\n",xIDorg[[i]],xIDorg[[j]],xIDorg[[j]],xIDorg[[i]],j,i ]]]
				]
			}]
		}];
		
		
		Close[file];
		Close[filedec];
		Close[filefun];
	]

(************************* Link Endpoint Kinematics  **********************)
(************************* Link Endpoint Kinematics  **********************)
(************************* Link Endpoint Kinematics  **********************)

LinkEndpointKinematics[infile_,outfile_]:= 
	
	(* now we start calculating the endpoint kinematics *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,{0,0,0},outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;
			  		
		(* the output file *)
		file    = StringJoin[outfile,"_LEKin_math.h"];
		filedec = StringJoin[outfile,"_LEKin_declare.h"];
		filecon = StringJoin[outfile,"_LEKin_contact.h"];
		OpenWrite[file];
		OpenWrite[filedec];
		OpenWrite[filecon];

		WriteString[file,"/* this function assumes that the array Xlink[nLinks+1][3+1]\n is passed as an argument. Only the real number of links are computed */\n\n"];
			  		
		(* fix the numerics *)
		rep = {3.->3, 2.->2, 1.->1, -1.-> -1, 0.->0};
			  		
		(* Note: ox... variables are output for C -- they are matched by
			a x... variable which contains the symbol which should be
			assigned, i.e, x... = ox... will be a proper C assignment *)
			  		
		(* precompute the sin and cos for each joint *)
		oxjsincos = Table[Flatten[{N[Sin[xj[[i,1]]]],N[Cos[xj[[i,1]]]]}(1-xisprismatic[[i]])],{i,1,xnj}];
		xjsincos = Table[MakeSinCos[xj[[i,1]]],{i,1,xnj}];
		WriteString[file,"/* sine and cosine precomputation */\n"];
		WriteOutput[file,xjsincos,oxjsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xjsincos],""];
			  		
		(* precompute the sin and cos for the rotation matrices *)
		oxrotsincos = 
			Table[Flatten[{Sin[xrot[[i,j]]],Cos[xrot[[i,j]]]}],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}];
		xrotsincos = Table[MakeSinCosRot[xrot[[i,j]]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}];
			
			  		   
		WriteString[file,"/* rotation matrix sine and cosine precomputation */\n"];
		WriteOutput[file,xrotsincos,oxrotsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xrotsincos],""];
									
			  		   
		(* need the inverse matrices only *)
		oxSinv = Table[
			RotMatInvSpec[xrotsincos[[i]]].
			RotMatInvSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error2],
			{j,1,3}]],
			{i,1,xnj}] /. rep;
		xSinv = Table[
			MakeSimplerMatrix[oxSinv[[i]],
			ToString[StringForm["Si``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0],
			{i,1,xnj}];
			
		(* rotation matrix for floating base *)
		oxSinv[[0]] = RotMatQuatInv[base[[2,{1,2,3,4}]]] /. rep;
		xSinv[[0]] = MakeSimplerMatrix[oxSinv[[0]],
			ToString[StringForm["Si``````",0,0,"[[``,``]]"]],0];
			
		WriteString[file,"/* inverse rotation matrices */\n"];
		WriteOutput[file,Zt[xSinv],Zt[oxSinv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xSinv]],"[3+1][3+1]"];
		
					  		
		(* Every joint needs to be pushed back into the world coordinate system.
		   The position of the joint is given by the translation vector of the 
		   joint. This translation vector is the start, and the rotations and
		   translations of all subsequent joints need to be applied. Just a big
		   nested mess. *)
		   
			       		  		
		WriteString[file,"/* calculation of link coordinates */\n"];
		
		(* note that the strange stringform->tostring->toexpression is needed
		   to avoid that Mathematica makes this a local variable *)
		WriteDeclaration[filedec,"double  ",{ToES[StringForm["v"]]},"[3+1]"];
		WriteDeclaration[filedec,"double  ",{ToES[StringForm["vv"]]},"[3+1]"];

		xIDorg[[0]]=0;
		xtrans[[0]] = {base[[2,5]],base[[2,6]],base[[2,7]]};
		countlinks = 0;
		properlinkflag = Table[0,{i,1,xnj}];
		properlinkflag[[0]] = 0;
		contactlist={};
		
		Do[
			{
			j=i;
			xv = xtrans[[j]];
			count =0;
			lastproperlink=0; (* the base is always the initial assumption of the last proper link *)
            lastproperlinkjoint=0; (* the joint ID that corresponds to the last proper link *)
			(* check whether there is a real translation happening in this joint, if yes, set flag=1 *)
			If [NumberQ[xv[[1]]] && NumberQ[xv[[2]]] && NumberQ[xv[[3]]],If[xv.xv==0,flag=0,flag=1],flag=1];
			If [j > 0 && xisprismatic[[xpred[[j]]]]==1,flag=1,Null,Null]; (* prismatic joints always trigger a flag=1 *) 
			If [xisprismatic[[j]]==1,flag=1,Null,Null];
			If[flag==1,
			{If[i==0,Null,countlinks = countlinks+1]; (* skip counting up for the base *)
			properlinkflag[[i]]=countlinks;
			While[NumberQ[xpred[[j]]], (* recursively go back and compute the transformations for this dof *)
				{
				j=xpred[[j]];
				If[lastproperlink==0,If[DummyLinkQ[xj[[j,3]]],NULL,{lastproperlink=properlinkflag[[j]],lastproperlinkjoint=j}],Null]; (* dummies cannot be proper links *)
				count = count+1;
				If[Mod[count,2] == 1,
					{oxv = xSinv[[j]].(xv+xj[[j,1]] xax[[j,{4,5,6}]])+xtrans[[j]] /. rep;
					xv = MakeSimplerVector[oxv,ToString[StringForm["v``","[[``]]"]],0];
					WriteOutput[file,xv,oxv]},
					{oxv = xSinv[[j]].(xv+xj[[j,1]] xax[[j,{4,5,6}]])+xtrans[[j]] /. rep;
					xv = MakeSimplerVector[oxv,ToString[StringForm["vv``","[[``]]"]],0];
					WriteOutput[file,xv,oxv]}
				]
				}
			];
			tt = MakeSimplerVector[xv,ToString[StringForm["Xlink[[``,``]]",countlinks,"``"]],1];
			WriteString[file,StringForm["/* link ``: `` */\n",countlinks,ToString[xtrans[[i]],InputForm]]];
			WriteOutput[file,tt,xv];

		    j=i;
			While[DummyLinkQ[xj[[j,3]]] && j != 0, j=xpred[[j]]]; (* catch dummy joints for contacts *)
            aux=xIDorg[[j]];
						
			WriteString[filecon,StringForm["contacts[``].base_dof_start=``;\n",countlinks,ToString[aux,OutputForm]]];
			WriteString[filecon,StringForm["contacts[``].base_dof_end=``;\n",countlinks,ToString[aux,OutputForm]]];
			If[i==0,Null,contactlist = Append[contactlist,xIDorg[[i]]]];
			
			active=0;
			WriteString[filecon,StringForm["contacts[``].active=``;\n",countlinks,ToString[active,OutputForm]]];			
			
			If[!DummyLinkQ[xj[[i,3]]],aux=countlinks,aux=lastproperlink]; 
			WriteString[filecon,StringForm["contacts[``].off_link_start=``;\n",countlinks,ToString[aux,OutputForm]]];
			WriteString[filecon,StringForm["contacts[``].off_link_end=``;\n",countlinks,ToString[aux,OutputForm]]];

            aux=0.5; 
			WriteString[filecon,StringForm["contacts[``].fraction_start=``;\n",countlinks,ToString[aux,OutputForm]]];
			WriteString[filecon,StringForm["contacts[``].fraction_end=``;\n",countlinks,ToString[aux,OutputForm]]];

			WriteString[filecon,StringForm["contacts[``].id_start=``;\n",countlinks,ToString[countlinks,OutputForm]]];
			WriteString[filecon,StringForm["contacts[``].id_end=``;\n\n",countlinks,ToString[countlinks,OutputForm]]];

			If[Apply[Plus,xsuc[[i]]]>0,lastproperlink=countlinks,Null];
			
			}
			,Null]
			},
		{i,0,xnj,1}];

        WriteString[filecon,StringForm["read_extra_contact_points(config_files[CONTACTS]);\n\n"]];

		
		Close[file];
		Close[filedec];
		Close[filecon];
		
		(* create also information about the jacobians of every contact point *)
		outfile2 = StringJoin[outfile,"_Contact"];
		GeometricJacobian[infile,contactlist,outfile2];
		(* obsolete files *)
		DeleteFile[StringJoin[outfile,"_Contact_Prismatic_Joints.h"]];	
		DeleteFile[StringJoin[outfile,"_Contact_Floating_Base.h"]];	
		DeleteFile[StringJoin[outfile,"_LEKin_declare.h"]];	
		DeleteFile[StringJoin[outfile,"_LEKin_math.h"]];	
		
	]

(************************* Link Rotation Matrix  **********************)
(************************* Link Rotation Matrix  **********************)
(************************* Link Rotation Matrix  **********************)

LinkRotationMatrix[infile_,endeffs_,outfile_]:= 
	
	(* now we start calculating the rotation matrix *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,{0,0,0},outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;
			  		
		(* the output file *)
		file    = StringJoin[outfile,"_LRMat_math.h"];
		filedec = StringJoin[outfile,"_LRMat_declare.h"];
		OpenWrite[file];
		OpenWrite[filedec];

		WriteString[file,"/* this function assumes that the array XRot[n_links+1][3+1][3+1]\n is passed as an argument */\n\n"];
			  		
		(* fix the numerics *)
		rep = {3.->3, 2.->2, 1.->1, -1.-> -1, 0.->0};
			  		
		(* Note: ox... variables are output for C -- they are matched by
			a x... variable which contains the symbol which should be
			assigned, i.e, x... = ox... will be a proper C assignment *)
			  		
		(* precompute the sin and cos for each joint *)
		oxjsincos = Table[Flatten[{N[Sin[xj[[i,1]]]],N[Cos[xj[[i,1]]]]}(1-xisprismatic[[i]])],{i,1,xnj}];
		xjsincos = Table[MakeSinCos[xj[[i,1]]],{i,1,xnj}];
		WriteString[file,"/* sine and cosine precomputation */\n"];
		WriteOutput[file,xjsincos,oxjsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xjsincos],""];
			  		
		(* precompute the sin and cos for the rotation matrices *)
		oxrotsincos = 
			Table[Flatten[{Sin[xrot[[i,j]]],Cos[xrot[[i,j]]]}],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}];
		xrotsincos = Table[MakeSinCosRot[xrot[[i,j]]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}];
			  		   
		WriteString[file,"/* rotation matrix sine and cosine precomputation */\n"];
		WriteOutput[file,xrotsincos,oxrotsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xrotsincos],""];
									
			  		   
		(* need the inverse matrices only *)
		oxSinv = Table[
			RotMatInvSpec[xrotsincos[[i]]].
			RotMatInvSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error2],
			{j,1,3}]],
			{i,1,xnj}] /. rep;
		xSinv = Table[
			MakeSimplerMatrix[oxSinv[[i]],
			ToString[StringForm["Si``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0],
			{i,1,xnj}];
			
		(* rotation matrix for floating base *)
		oxSinv[[0]] = RotMatQuatInv[base[[2,{1,2,3,4}]]] /. rep;
		xSinv[[0]] = MakeSimplerMatrix[oxSinv[[0]],
			ToString[StringForm["Si``````",0,0,"[[``,``]]"]],0];
			
		WriteString[file,"/* inverse rotation matrices */\n"];
		WriteOutput[file,Zt[xSinv],Zt[oxSinv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xSinv]],"[3+1][3+1]"];
		
					  		
		(* which joints are actually needed for the Matrix calculation *)
		jointlist={};
		For[i=1, i<=Length[endeffs], ++i,{
			(* replace the ID numbers in endeff with the local ID numbers *)
			jID = Flatten[Position[xIDorg,endeffs[[i]]]][[1]];
			jointlist=Flatten[Append[jointlist,jID]];
		 }
		];
		


		(* The specified joints needs to be pushed back into the world coordinate system.
		   Only rotation matrices matter, so this is rather easy. *)
			       		  		
		WriteString[file,"/* calculation of rotation matrices */\n"];
		
		(* note that the strange stringform->tostring->toexpression is needed
		   to avoid that Mathematica makes this a local variable *)
		WriteDeclaration[filedec,"double  ",{ToES[StringForm["v"]]},"[3+1][3+1]"];
		WriteDeclaration[filedec,"double  ",{ToES[StringForm["vv"]]},"[3+1][3+1]"];

		xIDorg[[0]]=0;
		
		Do[
			{
			j=jointlist[[i]];
			count =0;
			xv = xSinv[[j]];
			While[NumberQ[xpred[[j]]], (* recursively go back and compute the transformations for this dof *)
				{
				j=xpred[[j]];
				count = count+1;
				If[Mod[count,2] == 1,
					{oxv = xSinv[[j]].xv /. rep;
					xv = MakeSimplerMatrix[oxv,ToString[StringForm["v``","[[``,``]]"]],0];
					WriteOutput[file,xv,oxv]},
					{oxv = xSinv[[j]].xv /. rep;
					xv = MakeSimplerMatrix[oxv,ToString[StringForm["vv``","[[``,``]]"]],0];
					WriteOutput[file,xv,oxv]}
				]
				}
			];
			tt = MakeSimplerMatrix[xv,ToString[StringForm["XRot[[``,``]]",i,"``,``"]],1];
			WriteString[file,StringForm["/* Rotation Matrix: `` */\n",ToString[endeffs[[i]],InputForm]]];
			WriteOutput[file,tt,xv];
			
			
			},
		{i,1,Length[jointlist]}];
			  		
		Close[file];
		Close[filedec];
	]

(************************* OpenGLKinematics  **********************)
(************************* OpenGLKinematics  **********************)
(************************* OpenGLKinematics  **********************)

OpenGLKinematics[infile_,outfile_]:= 
	
	(* now we start calculating the endpoint kinematics *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,{0,0,0},outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;
			  		
		(* the output file *)
		file    = StringJoin[outfile,"_OpenGL.h"];
		OpenWrite[file];

		WriteString[file,"/* this function generates simple OpenGL graphics code to draw each link */\n\n"];
			  		
		(* Note: ox... variables are output for C -- they are matched by
			a x... variable which contains the symbol which should be
			assigned, i.e, x... = ox... will be a proper C assignment *)
			  		
		(* Starting from the base, each link needs to be drawn. Note that the
		   final object to be drawn as a link is left open as a user-specified
		   function, such that lines, wire-frames, or solids can be used. At each
		   joint, an additional object can be drawn (e.g., a sphere), to make
		   the graphics more appealing *)

		(* note that the strange stringform->tostring->toexpression is needed
		   to avoid that Mathematica makes this a local variable *)

		(* WriteString[file,"glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);\n\n"]; *)

		(* A recursive function to write the OpenGL statements *)
		
		WriteOpenGL[fn_,ID_]:= Block[{i,s},
			WriteString[fn,"\n"];
			s=ToString[StringForm["/* JointID = `` */\n\n",xIDorg[[ID]]]];
			WriteString[fn,s];
			WriteString[fn,"glPushMatrix();\n"];
			norm=Sqrt[xtrans[[ID]].xtrans[[ID]]];
			If [ NumberQ[norm] && norm == 0, 
				{
				WriteString[fn,"glPushMatrix();\n"];
				s=ToString[StringForm["myDrawGLElement((int)``,(double)``,(int)``);\n",xIDorg[[ID]],0,0]];
				WriteString[fn,FS[s]];
				WriteString[fn,"glPopMatrix();\n"];
				},
				{
				WriteString[fn,"glPushMatrix();\n"];
				s=ToString[StringForm["if (``==0 && ``==0)\n",CForm[xtrans[[ID,1]]],CForm[xtrans[[ID,2]]]]];
				WriteString[fn,FS[s]];
		  	  s=ToString[StringForm["glRotated((GLdouble)``,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);\n",CForm[180.0*(xtrans[[ID,3]]/norm-1.0)/2.]]];							  
				WriteString[fn,FS[s]];
				WriteString[fn,"else\n"];
				s=ToString[StringForm["glRotated((GLdouble)180.0,(GLdouble)``,(GLdouble)``,(GLdouble)``);\n",CForm[xtrans[[ID,1]]/2.],CForm[xtrans[[ID,2]]/2.],CForm[(xtrans[[ID,3]]+norm)/2.]]];			
				WriteString[fn,FS[s]];
				If[Length[xsuc[[ID]]]!=0,flag=1,flag=0];
				s=ToString[StringForm["myDrawGLElement((int)``,(double)``,(int)``);\n",xIDorg[[ID]],CForm[norm],flag]];
				WriteString[fn,FS[s]];
				WriteString[fn,"glPopMatrix();\n"];
				If[Length[xsuc[[ID]]]==0,Null,
					{s=ToString[StringForm["glTranslated((GLdouble)``,(GLdouble)``,(GLdouble)``);\n",CForm[xtrans[[ID,1]]],CForm[xtrans[[ID,2]]],CForm[xtrans[[ID,3]]]]];
					WriteString[fn,FS[s]]}];
			}];
			If[Length[xsuc[[ID]]]==0,Null,
			  {
			  If[ID==0,
			  	If[NumberQ[xrot[[ID]].xrot[[ID]]] && xrot[[ID]].xrot[[ID]]==0,Null,{
					s=ToString[StringForm["glRotated((GLdouble)``,(GLdouble)``,(GLdouble)``,(GLdouble)``);\n",CForm[2.*ArcCos[xrot[[ID,1]]]/Pi*180.],CForm[xrot[[ID,2]]],CForm[xrot[[ID,3]]],CForm[xrot[[ID,4]]]]];
					WriteString[fn,FS[s]];
					}
				  ];
			  	,
			  	Do[ If[ NumberQ[xrot[[ID,i]]] && xrot[[ID,i]]==0,Null,
					{
					s=ToString[StringForm["glRotated((GLdouble)``,(GLdouble)``,(GLdouble)``,(GLdouble)``);\n",CForm[xrot[[ID,i]]/Pi*180.],-Abs[Sign[1-i]]+1.,-Abs[Sign[2-i]]+1.,-Abs[Sign[3-i]]+1.]];
					WriteString[fn,FS[s]];
					}],{i,1,Length[xrot[[ID]]]}];
			  ];
			  If[ID==0,Null,
			  	{If[xisprismatic[[ID]]==1,
					s=ToString[StringForm["glTranslated((GLdouble)``,(GLdouble)``,(GLdouble)``);\n",xj[[ID,1]] xax[[ID,4]],xj[[ID,1]] xax[[ID,5]],xj[[ID,1]] xax[[ID,6]]]],
					s=ToString[StringForm["glRotated((GLdouble)``,(GLdouble)``,(GLdouble)``,(GLdouble)``);\n",CForm[xj[[ID,1]]/Pi*180.],xax[[ID,1]],xax[[ID,2]],xax[[ID,3]]]],
					Null];
			  	WriteString[fn,FS[s]];}];
			  }
			];
			Do[WriteOpenGL[fn,xsuc[[ID,i]]],{i,1,Length[xsuc[[ID]]]}];
			WriteString[fn,"glPopMatrix();\n"];
			];
			
		(* apply the recursive graphics generation *)
		xtrans[[0]] = {base[[2,5]],base[[2,6]],base[[2,7]]};
		xrot[[0]]   = {base[[2,1]],base[[2,2]],base[[2,3]],base[[2,4]]};
		xIDorg[[0]] = 0;
		xsuc[[0]]={};
		Do[If[xpred[[i]]==0,xsuc[[0]]=Append[xsuc[[0]],i],Null],{i,1,xnj}];
		WriteString[file,"\nglPushMatrix();\n"];
		WriteString[file,"myDrawGLElement((int)999,(double)0.0,(int)1);\n"];
		WriteString[file,"glPopMatrix();\n"];
		(* Do[If[xpred[[i]]==0,WriteOpenGL[file,i],Null],{i,1,xnj}]; *)
		WriteOpenGL[file,0];
		WriteString[file,"/*glutSwapBuffers();*/\n\n"];

			  		
		Close[file];
	]
	
(************************* Parameter Estimation  **********************)
(************************* Parameter Estimation  **********************)
(************************* Parameter Estimation  **********************)

(* parameter estimation for inverse dynamics *)
		
ParmEst[infile_,outfile_,gravity_]:= 

	(* now we start calculating the parameter estimation *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,gravity,outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;
			  		
		(* the output file *)
		file    = StringJoin[outfile,"_PE_math.h"];
		filedec = StringJoin[outfile,"_PE_declare.h"];
		OpenWrite[file];
		OpenWrite[filedec];
			  		
		(* fix the numerics *)
		rep = {1.->1, -1.-> -1, 0.->0, 2.->2, 3.->3};
			  		
		(* Note: ox... variables are output for C -- they are matched by
			a x... variable which contains the symbol which should be
			assigned, i.e, x... = ox... will be a proper C assignment *)
			  		
		(* precompute the sin and cos for each joint *)
		oxjsincos = Table[Flatten[{N[Sin[xj[[i,1]]]],N[Cos[xj[[i,1]]]]}(1-xisprismatic[[i]])],{i,1,xnj}];
		xjsincos = Table[MakeSinCos[xj[[i,1]]],{i,1,xnj}];
		WriteString[file,"/* sine and cosine precomputation */\n"];
		WriteOutput[file,xjsincos,oxjsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xjsincos],""];
			  		
		(* precompute the sin and cos for the rotation matrices *)
		oxrotsincos = 
			Table[Flatten[{Sin[xrot[[i,j]]],Cos[xrot[[i,j]]]}],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}];
		xrotsincos = Table[MakeSinCosRot[xrot[[i,j]]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}];
		WriteString[file,"/* rotation matrix sine and cosine precomputation */\n"];
		WriteOutput[file,xrotsincos,oxrotsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xrotsincos],""];
			  		   
		(* calculate the 3D rotation matrices *)
		xS = Table[
			RotMatSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error1],
			{j,1,3}]].
			RotMatSpec[xrotsincos[[i]]],
			{i,1,xnj}];

		(* rotation matrix for floating base *)
		xS[[0]] = RotMatQuat[base[[2,{1,2,3,4}]]];

			  		   
		(* need the inverse matrices as well *)
		xSinv = Table[
			RotMatInvSpec[xrotsincos[[i]]].
			RotMatInvSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error2],
			{j,1,3}]],
			{i,1,xnj}] /. rep;

		(* rotation matrix for floating base *)
		xSinv[[0]] = RotMatQuatInv[base[[2,{1,2,3,4}]]] /. rep;

			  		
		(* build the spatial rotation matrices *)
		xX = Table[
			Xtransspatial[xS[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]],
			{i,1,xnj}];
		oxXinv = Table[
			Xtransspatialinv[xSinv[[i]],xtrans[[i]],xj[[i,1]] xax[[i,{4,5,6}]]] /. rep,
			{i,1,xnj}];
		xXinv = Table[
			MakeSimplerMatrix[oxXinv[[i]],
			ToString[StringForm["Xinv$``$$$``",xID[[i]],"[[``,``]]"]],1],
			{i,1,xnj}];

		(* add the floating base in the zero element *)
		xX[[0]]    = Xtransspatial[xS[[0]],0*base[[2,{5,6,7}]],{0,0,0}];

		oxXinv[[0]] = Xtransspatialinv[xSinv[[0]],0*base[[2,{5,6,7}]],{0,0,0}];
        xXinv[[0]] = MakeSimplerMatrix[oxXinv[[0]],ToString[StringForm["Xinv$``$$$``",xID[[0]],"[[``,``]]"]],1];

		WriteString[file,"/* inverse spatial rotation matrices */\n"];
		WriteOutput[file,Zt[xXinv],Zt[oxXinv]];
		WriteString[filedec,ToString[StringForm["double  Xinv[``+1][6+1][6+1];\n\n",xnj]]];
			  		
		(* build the spatial rotation vectors and their transposes *)
		xs = Table[
			xax[[i]],
			{i,1,xnj}];
		oxst = Table[Flatten[{xs[[i,{4,5,6}]],xs[[i,{1,2,3}]]}],{i,1,xnj}];
		xst = Table[MakeSimplerVector[oxst[[i]],ToString[StringForm["st$``$$$``",xID[[i]],"[[``]]"]],1],
			{i,1,xnj}];
			WriteString[file,"/* spatial transpose of axis of the DOFs */\n"];
		WriteOutput[file,xst,oxst];
		WriteString[filedec,ToString[StringForm["double  st[``+1][6+1];\n\n",xnj]]];
			       		  		
		(* build the velocity vectors *)
		xv = {}; 
		oxv = {}; 

		xvbase      = Vel2Spatial[base[[2,{5,6,7}]]*0,base[[3,{5,6,7}]],base[[3,{8,9,10}]]];
		oxv[[0]]    = xX[[0]].xvbase;
		xv[[0]]     = MakeSimplerVector[oxv[[0]],ToString[StringForm["v````",0,"[[``]]"]],0];
		
		Do[
			{oxv=Append[oxv,Vispatial[xv[[xpred[[i]]]],xX[[i]],xs[[i]],xj[[i,2]]]] /. rep;
			xv=Append[xv,MakeSimplerVector[oxv[[i]],
			ToString[StringForm["v````",xID[[i]],"[[``]]"]],0]]},
			{i,1,xnj}];
		WriteString[file,"/* velocity vectors */\n"];
		WriteOutput[file,Zt[xv],Zt[oxv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xv]],"[6+1]"];
	
			  		   
		(* build the acceleration vectors *)
		xa = {}; 
		oxa = {}; 

		(* Note that this is a simplifiedverion of Acc2Spatial, which leaves out all cross products. *)
        (* xabase = Acc2Spatial[{0,0,0},base[[3,{5,6,7}]],base[[4,{5,6,7}]]-gravity,base[[3,{8,9,10}]],base[[4,{8,9,10}]]]; *)
	    xabase = Flatten[{base[[4,{8,9,10}]],base[[4,{5,6,7}]]-gravity}];
		oxa[[0]] = xX[[0]].xabase;
		xa[[0]] = MakeSimplerVector[oxa[[0]],ToString[StringForm["a````",xID[[0]],"[[``]]"]],0];
		Do[
			{oxa=Append[oxa,Aispatial[xa[[xpred[[i]]]],xX[[i]],xv[[i]],xs[[i]],xj[[i,2]],xj[[i,3]]]] /. rep;
			xa=Append[xa,MakeSimplerVector[oxa[[i]],
			ToString[StringForm["a````",xID[[i]],"[[``]]"]],0]]},
			{i,1,xnj}];
		WriteString[file,"/* acceleration vectors */\n"];
		WriteOutput[file,Zt[xa],Zt[oxa]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xa]],"[6+1]"];
			  		   
		(* compute the KinematicMatrices *)
		oxA = Table[FormKinematicMatrix[xv[[i]],xa[[i]]],{i,1,xnj}] /. rep;
		xA  = Table[
			MakeSimplerMatrix[oxA[[i]],
			ToString[StringForm["A$``$$$``",i,"[[``,``]]"]],1],
			{i,1,xnj}];

                (* add base *)
		oxA[[0]] = FormKinematicMatrix[xv[[0]],xa[[0]]];
        xA[[0]] = MakeSimplerMatrix[oxA[[0]],ToString[StringForm["A$``$$$``",0,"[[``,``]]"]],1];

		WriteString[file,"/* kinematic matrices */\n"];
		WriteString[file,ToString[StringForm["bzero((void *)A,sizeof(double)*(``+1)*(6+1)*(N_RBD_PARMS+1));\n",xnj]]];
		WriteOutput[file,Zt[xA],Zt[oxA]];
		WriteString[filedec,ToString[StringForm["double  A[``+1][6+1][N_RBD_PARMS+1];\n\n",xnj]]];
		
		(* the predecessor array needs to be written out *)
		oxP = xpred;
		xP  = Table[ToES[ToString[StringForm["pred[[``]]",i]]],{i,1,xnj}];
		WriteString[file,"/* predecessor vectors */\n"];
		WriteOutput[file,xP,oxP];
		WriteString[filedec,ToString[StringForm["int     pred[``+1];\n",xnj]]];
		
		(* the joint map: how to find the internal ID from the official ID *)
		oxM = xID;
		xM  = Table[ToES[ToString[StringForm["map[[``]]",xIDorg[[i]]]]],{i,1,xnj}];

		WriteString[file,"/* output mapping, i.e, which joint is where */\n"];
		WriteString[file,"map[0]=0;\n"];
		WriteOutput[file,xM,oxM];
		WriteString[filedec,ToString[StringForm["int     map[``+1];\n",Max[Ot[xIDorg]]]]];

        (* fixed base or floating base *)
		If[base[[6]]==0,floatingBase=0,floatingBase=1];
		
		(* a piece of plain C-code that generates the elements of the big K matrix *)
		WriteString[file,"{                                                           \n"];
		WriteString[file,"  int ii;                                                   \n"];
		WriteString[file,"  int jj;                                                   \n"];
		WriteString[file,"  int nn;                                                   \n"];
		WriteString[file,"  int mm;                                                   \n"];
		WriteString[file,"  int level;                                                \n"];
		WriteString[file,ToString[StringForm["  int xnj=``;                           \n",xnj]]];
		WriteString[file,ToString[StringForm["  int fbflag=``;                        \n",floatingBase]]];
		WriteString[file,"                                                            \n"];
		WriteString[file,"  for ( ii=xnj; ii>=0; --ii) {                              \n"];
		WriteString[file,"    level = ii;                                             \n"];
		WriteString[file,"    while (1) {                                             \n"];
		WriteString[file,"                                                            \n"];
		WriteString[file,"      if (level==ii)                                        \n"];
		WriteString[file,"        for (nn=1;nn<=6;++nn)                               \n"];
		WriteString[file,"          for (mm=1;mm<=N_RBD_PARMS;++mm)                   \n"];
		WriteString[file,"            U[nn][mm]=A[ii][nn][mm];                        \n"];
		WriteString[file,"      else                                                  \n"];
		WriteString[file,"        for (nn=1;nn<=6;++nn)                               \n"];
		WriteString[file,"          for (mm=1;mm<=N_RBD_PARMS;++mm)                   \n"];
		WriteString[file,"            U[nn][mm]=Unew[nn][mm];                         \n"];
		WriteString[file,"                                                            \n"];
        WriteString[file,"      if (level == 0) {                                     \n"];
		WriteString[file,"        for (nn=1;nn<=6;++nn)                               \n"];
		WriteString[file,"          for (mm=1;mm<=N_RBD_PARMS;++mm) {                 \n"];
		WriteString[file,"            Unew[nn][mm]=0;                                 \n"];
		WriteString[file,"            for (jj=1;jj<=6;++jj)                           \n"];
		WriteString[file,"              Unew[nn][mm]+=Xinv[level][nn][jj]*U[jj][mm];  \n"];
		WriteString[file,"          }                                                 \n"];
		WriteString[file,"        for (nn=1;nn<=6;++nn)                               \n"];
		WriteString[file,"          for (mm=1;mm<=N_RBD_PARMS;++mm)                   \n"];
		WriteString[file,"            K[xnj+nn][ii*N_RBD_PARMS+mm]=Unew[nn][mm]*fbflag;\n"];
		WriteString[file,"        break;                                              \n"];
        WriteString[file,"      } else {                                              \n"];
		WriteString[file,"        for (mm=1;mm<=N_RBD_PARMS;++mm) {                   \n"];
		WriteString[file,"          K[level][ii*N_RBD_PARMS+mm]=0;                    \n"];
		WriteString[file,"          for (nn=1;nn<=6;++nn)                             \n"];
		WriteString[file,"            K[level][ii*N_RBD_PARMS+mm]+=st[level][nn]*U[nn][mm];\n"];
		WriteString[file,"        }                                                   \n"];
		WriteString[file,"      }                                                     \n"];
		WriteString[file,"                                                            \n"];
		WriteString[file,"      for (nn=1;nn<=6;++nn)                                 \n"];
		WriteString[file,"        for (mm=1;mm<=N_RBD_PARMS;++mm) {                   \n"];
		WriteString[file,"          Unew[nn][mm]=0;                                   \n"];
		WriteString[file,"          for (jj=1;jj<=6;++jj)                             \n"];
		WriteString[file,"            Unew[nn][mm]+=Xinv[level][nn][jj]*U[jj][mm];    \n"];
		WriteString[file,"        }                                                   \n"];
		WriteString[file,"      level=pred[level];                                    \n"];
		WriteString[file,"    }                                                       \n"];
		WriteString[file,"  }                                                         \n"];
		WriteString[file,"}                                                           \n"];
		WriteString[file,"                                                            \n"];


		(* a K and U matrix a needed as temporary matrices *)
		WriteString[filedec,ToString[StringForm["double  K[``+1][``*N_RBD_PARMS+1];\n",xnj+6,(xnj+1)]]];
		WriteString[filedec,"double  U[6+1][N_RBD_PARMS+1];\n"];
		WriteString[filedec,"double  Unew[6+1][N_RBD_PARMS+1];\n"];
			  		
		(* at last, create the output elements of the regression: this is only necessary
		   because I reordered the numbering of the DOFS internally *)
		oxY = Table[xj[[i,4]],{i,1,xnj}];
                oxY = Flatten[Append[oxY,{0,0,0,0,0,0}]];
		xY = MakeSimplerVector[oxY,ToString[StringForm["Y``","[[``]]"]],0];
		WriteString[file,"/* The outputs that are associate with each row of K */\n"];
		WriteOutput[file,xY,oxY];
		WriteDeclaration[filedec,"double  ",Flatten[xY],ToString[StringForm["[``+1]",xnj+6]]];
		  
		Close[file];
		Close[filedec];
	]

(************************* Geometric Jacobian  **********************)
(************************* Geometric Jacobian  **********************)
(************************* Geometric Jacobian  **********************)

GeometricJacobian[infile_,endeffs_,outfile_]:= 
	
	(* now we start calculating the Jacobian *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,{0,0,0},outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;
  		
		(* the output file *)
		file    = StringJoin[outfile,"_GJac_math.h"];
		filedec = StringJoin[outfile,"_GJac_declare.h"];
		OpenWrite[file];
		OpenWrite[filedec];

		WriteString[file,"/* the Jacobian J[n_endeffector*6+1][n_joints+1] must be given */\n\n"];
			  		
		(* fix the numerics *)
		rep = {1.->1, -1.-> -1, 0.->0};
			  		
		(* Note: ox... variables are output for C -- they are matched by
			a x... variable which contains the symbol which should be
			assigned, i.e, x... = ox... will be a proper C assignment *)
		
		(* which joints are needed for the Jacobian calculation *)
		oxjs = Table[0,{i,1,Length[endeffs]},{j,1,xnj}];
		For[i=1, i<=Length[endeffs], ++i,{
			(* replace the ID numbers in endeff with the local ID numbers *)
			jID = Flatten[ Position[xIDorg,endeffs[[i]]] ][[1]];
			While[jID != 0,{
				jID=xpred[[jID]];
				If[jID != 0 && !(NumberQ[xj[[jID,3]]] && xj[[jID,3]] == 0), oxjs[[i,xIDorg[[jID]]]]=1,Null];
			  }
			];
		 	}
		];	
		
		xjs=MakeSimplerMatrix[oxjs,ToString[StringForm["Jlist``","[[``,``]]"]],1];
		WriteString[file,"/* indices of Jacobian */\n"];
		WriteOutput[file,xjs,oxjs];
		WriteString[filedec,ToString[StringForm["int  Jlist[``+1][``+1];\n\n",Length[endeffs],xnj]]];
			  		   			  		
		Close[file];
		Close[filedec];
	];

(************************* Link Information  **********************)
(************************* Link Information  **********************)
(************************* Link Information  **********************)

LinkInformation[infile_,outfile_]:= 
	
	(* now we start calculating the all necessary information *)
  
	Module[{xl,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,i,j,xisprismatic},
  
		(* Read the infile *)
		xl=ReadSpecFile[infile,{0,0,0},outfile];
			  	 
		(* sort out the elements of the specification list *)
  		{xnj,xID,xIDorg,xax,xtrans,xrot,xsuc,xpred,xinert,xmcm,xm,xj,xuext,base,xisprismatic} = xl;
			  		
		(* the output file *)
		file    = StringJoin[outfile,"_LInfo_math.h"];
		filedec = StringJoin[outfile,"_LInfo_declare.h"];
		OpenWrite[file];
		OpenWrite[filedec];

		WriteString[file,"/* Need [n_joints+1]x[3+1] matrices: Xorigin,Xmcog,Xaxis, and Xlink[nLinks+1][3+1] */\n\n"];
			  		
		(* fix the numerics *)
		rep = {1.->1, -1.-> -1, 0.->0};
			  		
		(* Note: ox... variables are output for C -- they are matched by
			a x... variable which contains the symbol which should be
			assigned, i.e, x... = ox... will be a proper C assignment *)
			  		
		(* precompute the sin and cos for each joint *)
		oxjsincos = Table[Flatten[{N[Sin[xj[[i,1]]]],N[Cos[xj[[i,1]]]]}(1-xisprismatic[[i]])],{i,1,xnj}];
		xjsincos = Table[MakeSinCos[xj[[i,1]]],{i,1,xnj}];
		WriteString[file,"/* sine and cosine precomputation */\n"];
		WriteOutput[file,xjsincos,oxjsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xjsincos],""];
			  		
		(* precompute the sin and cos for the rotation matrices *)
		oxrotsincos = 
			Table[Flatten[{Sin[xrot[[i,j]]],Cos[xrot[[i,j]]]}],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}];
		xrotsincos = Table[MakeSinCosRot[xrot[[i,j]]],
			{i,1,xnj},{j,1,Length[xrot[[i]]]}];
		WriteString[file,"/* rotation matrix sine and cosine precomputation */\n"];
		WriteOutput[file,xrotsincos,oxrotsincos];
		WriteDeclaration[filedec,"double  ",Flatten[xrotsincos],""];
					  		   
		(* create inverse homogeneous transformation matrices *)
		oxSinv = Table[
			RotMatInvSpec[xrotsincos[[i]]].
			RotMatInvSpec[Table[If[Abs[xax[[i,j]]]==1,{xjsincos[[i,1]]*xax[[i,j]],xjsincos[[i,2]]},{0,1},error2],
			{j,1,3}]],
			{i,1,xnj}] /. rep;
		oxSinv[[0]] = RotMatQuatInv[base[[2,{1,2,3,4}]]] /. rep;

		(* make this a proper homogeneous transformation matrix *)	
		oxHinv = Table[
			Append[
			Table[Flatten[{oxSinv[[i,j]],xtrans[[i,j]]}],{j,1,3}],
			{0,0,0,1}].Transpose[{{1,0,0,0},{0,1,0,0},{0,0,1,0},Append[xj[[i,1]] xax[[i,{4,5,6}]],1]}],
			{i,1,xnj}] /. rep;
		oxHinv[[0]] = Append[
			Table[Flatten[{oxSinv[[0,j]],base[[2,4+j]]}],{j,1,3}],
			{0,0,0,1}];
			
		xHinv = Table[
			MakeSimplerMatrix[oxHinv[[i]],
			ToString[StringForm["Hi``````",xpred[[i]],xID[[i]],"[[``,``]]"]],0],
			{i,1,xnj}];
		xHinv[[0]] = MakeSimplerMatrix[oxHinv[[0]],
			ToString[StringForm["Hi``````",0,0,"[[``,``]]"]],0];
		WriteString[file,"/* inverse homogeneous rotation matrices */\n"];
		WriteOutput[file,Zt[xHinv],Zt[oxHinv]];
		WriteDeclaration[filedec,"double  ",Flatten[Zt[xHinv]],"[4+1][4+1]"];
		
		(* create all homogeneous transformations between the base coordinate system
		and every joint coordinate system *)		
		
		oxAinv      = Table[IdentityMatrix[4],{i,1,xnj}];
		oxAinv[[0]] = xHinv[[0]];
		xAinv       = oxAinv;
		xtrans[[0]] = {base[[2,5]],base[[2,6]],base[[2,7]]};

		Do [{
			oxAinv[[i]] = xAinv[[xpred[[i]]]].xHinv[[i]] /. rep;
			xAinv[[i]]  = MakeSimplerMatrix[oxAinv[[i]],
				ToString[StringForm["Ai0````",xID[[i]],"[[``,``]]"]],0]},
		{i,1,xnj}];
		WriteString[file,"/* per link inverse homogeneous rotation matrices */\n"];
		WriteOutput[file,Ot[xAinv],Ot[oxAinv]];
		WriteDeclaration[filedec,"double  ",Flatten[Ot[xAinv]],"[4+1][4+1]"];
		
		(* with the help of the xAinv matrices, all required information about each
		link can be computed by a simple matrix multiplication *)
		
		countlinks = -1;
		Do [{
			(* the origin vectors *)
			oxorigin = xAinv[[i]].{0,0,0,1} /. rep;
			oxorigin = Delete[oxorigin,4];
			xorigin  = MakeSimplerVector[oxorigin,ToString[StringForm["Xorigin[[``,``]]",xIDorg[[i]],"``"]],1];
					
			If [ xax[[i]].xax[[i]] == 0 && i != 0, Null, (* exclude dummy joints *)
			{

			WriteString[file,StringForm["/* joint ID: `` */\n",xIDorg[[i]]]];

			WriteOutput[file,xorigin,oxorigin];

			(* the mass*cog vectors *) 
			oxmcog = xAinv[[i]].Append[xmcm[[i]],xm[[i]]] /. rep;
			oxmcog = Delete[oxmcog,4];
			xmcog  = MakeSimplerVector[oxmcog,ToString[StringForm["Xmcog[[``,``]]",xIDorg[[i]],"``"]],1];
			WriteOutput[file,xmcog,oxmcog];
			
			(* the rotation axes vector -- need to zero 4th element in hom.vec to avoid translation component -- only want unit vector *)
			oxaxis = If[xisprismatic[[i]]==1, xAinv[[i]].Append[xax[[i,{4,5,6}]],0], xAinv[[i]].Append[xax[[i,{1,2,3}]],0],Null] /. rep;
			oxaxis = Delete[oxaxis,4];
			xaxis  = MakeSimplerVector[oxaxis,ToString[StringForm["Xaxis[[``,``]]",xIDorg[[i]],"``"]],1];
			WriteOutput[file,xaxis,oxaxis];

			(* the homogenious transformation matrix per DOF *)
			tt = MakeSimplerMatrix[xAinv[[i]],ToString[StringForm["Ahmatdof[[``,``]]",xIDorg[[i]],"``,``"]],1];
			WriteOutput[file,tt,xAinv[[i]]];
			
			
			},error19];
			
			
			(* is this a proper link, as indicated by a non-zero translation vector ?*)
			If [i==0 || Norm[xtrans[[i]]] != 0,flag = 1,flag = 0, flag = 1];

			If [i > 0 && xisprismatic[[xpred[[i]]]]==1,flag=1,Null,Null]; (* prismatic joints always trigger a flag=1 *) 
			If [xisprismatic[[i]]==1,flag=1,Null,Null];

			If [flag == 1,
			{
				countlinks = countlinks + 1;

				tt = MakeSimplerVector[oxorigin,ToString[StringForm["Xlink[[``,``]]",countlinks,"``"]],1];
				WriteString[file,StringForm["/* link ``: `` */\n",countlinks,ToString[xtrans[[i]],InputForm]]];
				WriteOutput[file,tt,oxorigin];

				(* pick the homogenous trans. such that all following zero translation joints are include *)
				j = i;
				While [Length[xsuc[[j]]]==1,
					If [Norm[xtrans[[ xsuc[[j,1]] ]]] == 0, j=xsuc[[j,1]], Break[], Break[]]
				];
								
				tt = MakeSimplerMatrix[xAinv[[j]],ToString[StringForm["Ahmat[[``,``]]",countlinks,"``,``"]],1];
				WriteOutput[file,tt,xAinv[[j]]];
				
			},
			error22];
			},
		{i,0,xnj}];		
			  		
		Close[file];
		Close[filedec];
	];


(* finish the package *)

End[]
EndPackage[]
