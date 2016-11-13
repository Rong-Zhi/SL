#include "pmps/bsxfun.h"


double my_Plus   (double a, double b) {
	return a + b;
}


double my_Minus   (double a, double b) {
	return a - b;
}


double my_Times (double a, double b) {
	return a * b;
}




int bsxfun_malloc ( Matrix a_in, Matrix b_in, Matrix* c_out ) {

	int a_rows = (int) a_in[0][NR];
	int a_cols = (int) a_in[0][NC];

	int b_rows = (int) b_in[0][NR];
	int b_cols = (int) b_in[0][NC];

	if ( a_rows != 1 && b_rows != 1 && a_rows != b_rows)
		return FALSE;

	if ( a_cols != 1 && b_cols != 1 && a_cols != b_cols)
		return FALSE;

//	if ( a_rows == 1 && b_cols!=1 && a_cols != b_cols )
//		return FALSE;
//
//	if ( a_cols == 1 && b_rows!=1 && a_rows != b_rows )
//		return FALSE;

	int c_rows = a_rows > b_rows ? a_rows : b_rows;
	int c_cols = a_cols > b_cols ? a_cols : b_cols;

	*c_out = my_matrix(1,c_rows,1,c_cols);

	return TRUE;

}




int bsxfun ( bsxfunOp op, Matrix a_in, Matrix b_in,   /* input arguments */
             Matrix c_out, 						      /* output arguments */
             bsxfunElemOp a_op, bsxfunElemOp b_op )   /* optional arguments, can be NULL */
{


	int a_rows = (int) a_in[0][NR];
	int a_cols = (int) a_in[0][NC];

	int b_rows = (int) b_in[0][NR];
	int b_cols = (int) b_in[0][NC];

	if ( a_rows != 1 && b_rows != 1 && a_rows != b_rows)
		return FALSE;

	if ( a_cols != 1 && b_cols != 1 && a_cols != b_cols)
		return FALSE;

//	if ( a_rows == 1 && b_cols!=1 && a_cols != b_cols )
//		return FALSE;
//
//	if ( a_cols == 1 && b_rows!=1 && a_rows != b_rows )
//		return FALSE;


	int c_rows = a_rows > b_rows ? a_rows : b_rows;
	int c_cols = a_cols > b_cols ? a_cols : b_cols;

	int i,j;
	for ( i = 1; i <= c_rows; ++i ) {

		int idx_a_r, idx_a_c, idx_b_r, idx_b_c;

		idx_a_r = c_rows > a_rows ? 1 : i;
		idx_b_r = c_rows > b_rows ? 1 : i;

		for ( j = 1; j <= c_cols; ++j ) {

			idx_a_c = c_cols > a_cols ? 1 : j;
			idx_b_c = c_cols > b_cols ? 1 : j;

			double a_el = a_in[idx_a_r][idx_a_c];
			a_el = a_op == 0 ? a_el : a_op ( a_el );

			double b_el = b_in[idx_b_r][idx_b_c];
			b_el = b_op == 0 ? b_el : b_op ( b_el );

			c_out[i][j] = op ( a_el, b_el );
		}
	}


	return TRUE;

}
