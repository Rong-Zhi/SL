/***********************************************************************
 * Reference   : Jens Kober et al, 
 *               Movement Templates for Learning of Hitting and Batting
 **********************************************************************/

#ifndef DDMP_H
#define DDMP_H

#include "utility.h"


/* DMP in the canonical dynamical system. */
typedef struct ddmp_x {

	char name[30];

	double alpha_x; // coefficient
	double alpha_h; // coefficient for the moving target
	double x; // phase
	double xd; // derivative of the phase

	int n_bases; // number of bases

	Vector c; // centers of the bases
	Vector h; // heights of the bases
	Vector psi; // bases

	double dt;
	double tau;
	
} ddmp_x_s;


/* DMP. */
typedef struct ddmp {

	char name[30];

	double alpha_z; // coefficient
	double beta_z; // coefficient

	double z; // DMP position (without temporal scaling)
	double zd; // DMP velocity (without temporal scaling)

	double y; // current position
	double yd; // current velocity
	double ydd; // current acceleration

	double g; // desired position
	double gf; // desired position used for the amplitude
	double gd; // desired velocity

	double y0; // initial position

	double a;
	double amp; // amplitude of the forcing function

	Vector w; // weights

	ddmp_x_s* using_x; // the canonical system used by the DMP

} ddmp_s;


//For initialing the structure and reserve memory
int ddmp_x_init ( ddmp_x_s* ID_x, int n_bases, char name[30], double dt );

//Resets state. Must be called when parameters are changed!
int ddmp_x_reset ( ddmp_x_s* ID_x );

//De-allocates private used memory (not the structure itself)
int ddmp_x_free ( ddmp_x_s* ID_x );

//Executes one time step of the canonical system
int ddmp_x_step ( ddmp_x_s* ID_x );



//For initialing the structure and reserve memory
int ddmp_init ( ddmp_s* ID, ddmp_x_s* ID_x, char name[30] );

//Resets state. Must be called when parameters are changed!
int ddmp_reset ( ddmp_s* ID );

//De-allocates private used memory (not the structure itself)
int ddmp_free ( ddmp_s* ID );

//Executes one time step of the primitive
int ddmp_step ( ddmp_s* ID );



//Printing
int ddmp_x_print ( ddmp_x_s* ID_x );
int ddmp_print ( ddmp_s* ID );


#endif
