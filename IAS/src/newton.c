#include "newton.h"

#include "stdio.h"
#include <math.h>
#include "SL.h"
#include "SL_common.h"



static const double TOLX = 1.0e-7;
static const double ALPHA = 1.0e-4;
static const double TOLF = 1.0e-4;
static const double STPMX = 100.0;
static const double MAXITS = 200;
static const double TOLMIN = 1.0e-6;



static inline float floatMax ( float a, float b ) {
	return (a > b ? a : b);
}

/******************************************************************
 Name :		fMin
 Description: 	returns f = 1/2*F*F at x
 *******************************************************************
 Parameter:
 Vector 	x(i)	guess for a root
 ******************************************************************/
static float fMin ( Vector x, Vector param, newFuncP_t nrfuncv, int nn, Vector fvec  ) {
	int i;
	float sum;

	(*nrfuncv)( nn, param, x, fvec );
	for ( sum = 0.0, i = 1; i <= nn; i++ )
		sum += sqr( fvec[i] );
	return 0.5 * sum;
}

/*****************************************************************
 Name:		lnsrch
 Description:	computes and evaluates the step width
 ******************************************************************
 Parameter:
 int	dim(i)		dimenstion of the parameter vector
 Vector	xold(i)		dim-dimensional point (evaluating point)
 float	fold(i)		value of the function
 Vector	g(i)		value of the gradient of the function
 Vector	p(i)		Newton direction
 Vector 	x(o)		new point
 float *	f(o)		new function value
 float	stpmax(i)	maximal step width
 int *	check(o)	false: normal exit
 true: x is too close to xold
 float	(*func)(Vector)	function
 *****************************************************************/
static int lnsrch ( int dim,
					Vector xold,
					float fold,
					Vector g,
					Vector p,
					Vector x,
					Vector param,
					float *f,
					float stpmax,
					int *check,
					newFuncP_t vecFP,
					float(*func) ( Vector, Vector, newFuncP_t , int, Vector ),
					Vector fvec) {
	int i;
	float a, b, l, l2, lmin, f2, slope, sum, tmp, test, l_tmp, rhs1, rhs2, disc;

	l2 = f2 = 0.0;

	*check = 0;

	for ( sum = 0.0, i = 1; i <= dim; i++ )
		sum += p[i] * p[i];
	sum = sqrt( sum );
	if ( sum > stpmax )
		for ( i = 1; i <= dim; i++ )
			p[i] *= stpmax / sum;

	for ( slope = 0.0, i = 1; i <= dim; i++ )
		slope += g[i] * p[i];
	if ( slope >= 0.0 ) {
		printf( "\n\nError occured in newton -> lnsrch \n\n" );
		return 0;
	}

	test = 0.0;
	for ( i = 1; i <= dim; i++ ) {
		tmp = fabs( p[i] ) / floatMax( fabs( xold[i] ), 1.0 );
		if ( tmp > test )
			test = tmp;
	}
	lmin = TOLX / test;
	l = 1.0;

	while ( TRUE ) {
		for ( i = 1; i <= dim; i++ )
			x[i] = xold[i] + l * p[i];
		*f = (*func)( x, param, vecFP , dim, fvec );

		if ( l < lmin ) {
			for ( i = 1; i <= dim; i++ )
				x[i] = xold[i];
			*check = 1;
			return 1;
		}
		else if ( *f <= fold + ALPHA * l * slope )
			return 1;
		else {
			if ( l == 1.0 )
				l_tmp = -slope / (2.0 * (*f - fold - slope));
			else {
				rhs1 = *f - fold - l * slope;
				rhs2 = f2 - fold - l2 * slope;
				a = (rhs1 / sqr( l ) - rhs2 / sqr( l2 )) / (l - l2);
				b = (-l2 * rhs1 / sqr( l ) + l * rhs2 / sqr( l2 )) / (l - l2);
				if ( a == 0.0 )
					l_tmp = -slope / (2.0 * b);
				else {
					disc = sqr( b ) - 3.0 * a * slope;
					if ( disc < 0.0 )
						l_tmp = 0.5 * l;
					else if ( b <= 0.0 )
						l_tmp = (-b + sqrt( disc )) / (3.0 * a);
					else
						l_tmp = -slope / (b + sqrt( disc ));
				}
				if ( l_tmp > 0.5 * l )
					l_tmp = 0.5 * l;
			}
		}

		l2 = l;
		f2 = *f;
		l = floatMax( l_tmp, 0.1 * l );
	}
}


/********************************************************************
 *********************************************************************
 Name: 		newt
 Description: 	Find the root by a globally convergent Newton's
 method

 ATTENTION : Be aware that this code only works for the table tennis ball optimization (because of the derivative).
 I still need to write it more generally

 ********************************************************************
 Parameters:
 Vector 	x(i)		initial guess for a root
 int 	dim(i)		dimension of x
 int *	check(o)	false on a normal return
 true if the routine has converged to
 a local minimum
 vecfunc(n,x,fvec)(i)	user supplied routine returning the
 vector of functions to be zeroed

 Returns:
 0	method failed
 1	method succeded
 ********************************************************************/
int newt ( 	Vector x,
			Vector param,
			int dim,
			int *check,
			newFuncP_t vecfunc,
			newFunc_J_P_t bvjac ) {


	static Vector g = 0;
	static Vector p = 0;
	static Vector xold = 0;
	static Vector fvec = 0;
	static Matrix J = 0;


	static int prev_dim = -1;
	if ( prev_dim != dim ) {
		prev_dim = dim;

		if ( g != 0 )
			free(g);
		if ( p != 0 )
			free(p);
		if ( xold != 0 )
			free(xold);
		if ( fvec != 0 )
			free(fvec);
		if ( J != 0 )
			free(J);

		g = my_vector( 1, dim );
		p = my_vector( 1, dim );
		xold = my_vector( 1, dim );
		fvec = my_vector( 1, dim );

		J = my_matrix( 1, dim, 1, dim );
	}

	int i, it, j;
	float  den, f, fold, stpmax, sum, tmp, test;


	//initialize
	test = 0.0;
	f = fMin( x, param, vecfunc ,dim, fvec );

	// test initial guess
	for ( i = 1; i <= dim; i++ )
		if ( fabs( fvec[i] ) > test )
			test = fabs( fvec[i] );

	if ( test < 0.01 * TOLF ) {
		*check = 3;
		return 1;
	}

	// set maximal step width
	for ( sum = 0.0, i = 1; i <= dim; i++ )
		sum += sqr( x[i] );
	stpmax = STPMX * floatMax( sqrt( sum ), (float) dim );

	// loop
	for ( it = 1; it <= MAXITS; it++ ) {
		//compute and evaluate Jacobian
		bvjac( x, J, param );
		//fdjac(dim,x,fvec,J,vecfunc);

		// compute gradient
		for ( i = 1; i <= dim; i++ ) {
			for ( sum = 0.0, j = 1; j <= dim; j++ )
				sum += J[j][i] * fvec[j];
			g[i] = sum;
		}

		for ( i = 1; i <= dim; i++ )
			xold[i] = x[i];
		fold = f;
		for ( i = 1; i <= dim; i++ )
			p[i] = -fvec[i];
		//	print_mat("J",J);
		//	print_vec("p", p);
		if ( !my_inv_ludcmp_solve( J, p, dim, p ) ) {
			print_vec( "x", x );
			print_vec( "param", param );
			printf( "Error occured in newt.\n" );
			return 0;
		}

		// linesearch to find a good step width
		if ( !lnsrch( dim, xold, fold, g, p, x, param, &f, stpmax, check, vecfunc, fMin, fvec ) ) {
			print_vec( "g", g );
			print_vec( "p", p );
			print_vec( "x", x );
			print_mat( "J", J );
		}

		// test for convergence on function values
		test = 0.0;
		for ( i = 1; i <= dim; i++ )
			if ( fabs( fvec[i] ) > test )
				test = fabs( fvec[i] );
		if ( test < TOLF ) {
			*check = 4;
			return 1;
		}

		if ( *check ) {
			test = 0.0;
			den = floatMax( f, 0.5 * dim );
			for ( i = 1; i <= dim; i++ ) {
				tmp = fabs( g[i] ) * floatMax( fabs( x[i] ), 1.0 ) / den;
				if ( tmp > test )
					test = tmp;
			}
			*check = (test < TOLMIN ? 1 : 5);
			return 1;
		}

		test = 0.0;
		for ( i = 1; i <= dim; i++ ) {
			tmp = (fabs( x[i] - xold[i] )) / floatMax( fabs( x[i] ), 1.0 );
			if ( tmp > test )
				test = tmp;
		}
		if ( test < TOLX ) {
			return 1;
		}
	}

	printf( "Maximal number of iterations exceeded in newt.\n" );
	return 0;

}
