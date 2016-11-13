/***********************************************************************
 * Reference   : Jens Kober et al, 
 *               Movement Templates for Learning of Hitting and Batting
 **********************************************************************/

#include "ddmp.h"

#include "math.h"
#include "string.h"


int ddmp_x_init ( ddmp_x_s* ID_x, int n_bases, char name[30], double dt )
{
	int i;
	double t = 0;

	strcpy(ID_x->name, name);

	ID_x->alpha_x = 25.0 / 3.0; // default value, if necessary change from task
	ID_x->alpha_h = 0.5;
	ID_x->n_bases = n_bases;
	
	// evalute the centers of the basis functions
	ID_x->c = my_vector( 1, n_bases );
	for ( i = 1; i <= n_bases; i++ ) {
		ID_x->c[i] = exp( -ID_x->alpha_x * t );
		t += 0.5 / (n_bases - 1); // bases are uniformly distributed
	}

	// evalute the heights of the basis functions
	ID_x->h = my_vector( 1, n_bases );
	for ( i = 1; i <= (n_bases - 1); i++ )
		ID_x->h[i] = 0.5 / sqr( (ID_x->c[i + 1] - ID_x->c[i]) * 0.65 );
	ID_x->h[n_bases] = ID_x->h[n_bases - 1];

	ID_x->psi = my_vector( 1, n_bases );

	ID_x->dt = dt;

	ddmp_x_reset ( ID_x );

	return TRUE;
}


int ddmp_x_reset ( ddmp_x_s* ID_x ) {
	ID_x->x = 1.0; // reset the phase to 1
	ID_x->xd = 0.0; // reset the phase velocity to 0
	vec_zero( ID_x->psi ); // reset the bases to 0
	return TRUE;
}


int ddmp_x_step ( ddmp_x_s* ID_x ) {
	int i;

	// evaluate bases
	for ( i = 1; i <= ID_x->n_bases; i++ )
		ID_x->psi[i] = exp( -ID_x->h[i] * sqr( ID_x->x - ID_x->c[i] ) );

	// compute current velocity
	ID_x->xd = -ID_x->alpha_x * ID_x->x * ID_x->tau;

	// execute one step and update the phase
	ID_x->x = ID_x->x + ID_x->dt * ID_x->xd;

	return TRUE;
}


int ddmp_x_free ( ddmp_x_s* ID_x ) {
	if ( ID_x->c != 0)
		free(ID_x->c);
	if ( ID_x->c != 0)
		free(ID_x->h);
	if ( ID_x->c != 0)
		free(ID_x->psi);
	return TRUE;
}


////////////////////////////////////////////////////////////////////////


int ddmp_init ( ddmp_s* ID, ddmp_x_s* ID_x, char name[30] ) {
	strcpy( ID->name, name );
	ID->alpha_z = 25.0; // default value, if necessary change from task
	ID->beta_z = ID->alpha_z / 4.0; // default value, if necessary change from task
	ID->using_x = ID_x;
	ID->w = my_vector( 1, ID_x->n_bases );
	
	ID->g = 0.0;
	ID->gf = 0.0;
	ID->gd = 0.0;

	ID->y0 = 0.0;
	ID->a = 0.0;
	ID->amp = 0.0;
	vec_zero( ID->w );

	ddmp_reset( ID );

	return TRUE;
}

int ddmp_reset ( ddmp_s* ID ) {
	ID->z = 0.0;
	ID->zd = 0.0;

	ID->y = ID->y0;
	ID->yd = 0.0;
	ID->ydd = 0.0;
	
	ID->amp = ID->gf - ID->y0 + ID->a;
	ID->g = ID->gf - ID->gd * ID->using_x->alpha_h;

	return TRUE;
}


int ddmp_step ( ddmp_s* ID ) {
	int i;
	double f, sum_d;
	double sum_n = 0;

	ddmp_x_s* ID_x = ID->using_x;

	ID->g += ID->gd * ID_x->dt;

	// compute forcing function
	for ( i = 1; i <= ID_x->n_bases; i++ )
		sum_n += ID_x->psi[i] * ID->w[i] * ID_x->x;
	vec_sum( ID_x->psi, &sum_d );
	f = sum_n / (sum_d + ID_x->psi[0] * 1.e-10);

	ID->zd = (ID->alpha_z
			* (ID->beta_z * (ID->g - ID->y) + (ID->gd / ID_x->tau - ID->z))
			+ ID->amp * f) * ID_x->tau;
	
	ID->yd = ID->z * ID_x->tau;
	ID->ydd = ID->zd * ID_x->tau;

	ID->z = ID->z + ID_x->dt * ID->zd;
	ID->y = ID->y + ID_x->dt * ID->yd;

	return TRUE;
}


int ddmp_free ( ddmp_s* ID ) {
	if ( ID->w != 0)
		free(ID->w);
	return TRUE;
}


////////////////////////////////////////////////////////////////////////


int ddmp_x_print ( ddmp_x_s* ID_x ) {
	printf( "name   : %s \n", ID_x->name );
	printf( "alpha_x: %f \n", ID_x->alpha_x );
	printf( "alpha_h: %f \n", ID_x->alpha_h );

	printf( "x      : %f \n", ID_x->x );
	printf( "xd     : %f \n", ID_x->xd );

	printf( "basis  : %d \n", ID_x->n_bases );
	print_vec( "c", ID_x->c );
	print_vec( "h", ID_x->h );
	print_vec( "psi", ID_x->psi );

	printf( "dt      : %f \n", ID_x->dt );
	printf( "tau     : %f \n", ID_x->tau );

	return TRUE;
}


int ddmp_print ( ddmp_s* ID ) {
	printf( "name   : %s \n\n", ID->name );

	printf( "alpha_z: %f \n", ID->alpha_z );
	printf( "beta_z : %f \n\n", ID->beta_z );

	printf( "z      : %f \n", ID->z );
	printf( "zd     : %f \n\n", ID->zd );

	printf( "y      : %f \n", ID->y );
	printf( "yd     : %f \n", ID->yd );
	printf( "ydd    : %f \n\n", ID->ydd );

	printf( "g      : %f \n", ID->g );
	printf( "gf     : %f \n", ID->gf );
	printf( "gd     : %f \n\n", ID->gd );

	printf( "y0     : %f \n", ID->y0 );

	printf( "a      : %f \n", ID->a );
	printf( "using_x: %p \n\n", ID->using_x );

	print_vec( "w", ID->w );

	return TRUE;
}
