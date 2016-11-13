#include "ias_matrix_utilities.h"

#include <math.h>

int fread_mat_ascii ( char *filename, Matrix *new_matrix, int numCols ) {
	// determine number of rows
	FILE *file = fopen( filename, "r" );
	if ( file == NULL ) {
		printf( "Problem reading file %s!\n", filename );
		return -1;
	}

	char line[500]; /* or other suitable maximum line size */
	int numRows = 0;
	while ( fgets( line, sizeof(line), file ) != NULL ) /* read a line */
	{
		numRows++;
	}

	fseek( file, 0L, SEEK_SET );

	int rowNumber = 1;

	int j = 0;
	*new_matrix = my_matrix( 1, numRows, 1, numCols );
	for ( rowNumber = 1; rowNumber <= numRows; rowNumber++ ) {
		double tmp;
		for ( j = 1; j <= numCols; j++ ) {
			if ( fscanf( file, "%lf ", &tmp ) != 1 ) {
				return -1; //TODO dealloc matrix
			}
			(*new_matrix)[rowNumber][j] = tmp;
		}
		if ( fscanf( file, "\n" ) )
			return -1;
	}
	fclose( file );
	return numRows;
}


int malloc_mat_ascii ( char *filename, Matrix *new_matrix ) {

	int i,j;
	double tmp;

	int numRows = 0, numCols = 0;

	FILE *file = fopen( filename, "r" );
	if ( file == NULL ) {
		printf( "Problem reading file %s!\n", filename );
		return FALSE;
	}

	// determine number of rows
	char line[25000]; /* or other suitable maximum line size */
	while ( fgets( line, sizeof(line), file ) != NULL ) /* read a line */
		++numRows;


	fseek( file, 0L, SEEK_SET );


	// determine number of columns
	while ( fscanf( file, "%le", &tmp ) == 1 )
		++numCols;

	numCols = numCols / numRows;


	fseek( file, 0L, SEEK_SET );
	*new_matrix = my_matrix( 1, numRows, 1, numCols );

	for ( i = 1; i <= numRows; i++ ) {
		for ( j = 1; j <= numCols; j++ ) {
			if ( fscanf( file, "%le", &tmp ) != 1 ) {
				my_basic_free_matrix( *new_matrix );
				*new_matrix = 0;
				return FALSE;
			}
			(*new_matrix)[i][j] = tmp;
		}
	}
	fclose( file );
	return TRUE;
}


int fwrite_mat_ascii ( char *filename, Matrix matrix ) {
	// determine number of rows
	FILE *file = fopen( filename, "w" );
	if ( file == NULL ) {
		printf( "Problem opening file for writing %s!\n", filename );
		return 0;
	}

	int rows = matrix[0][NR];
	int col = matrix[0][NC];

	int i, j;
	for ( i = 1; i <= rows; ++i ) {
		for ( j = 1; j <= col; ++j )
			fprintf( file, "%lf ", matrix[i][j] );
		fprintf( file, "\n" );
	}

	fclose( file );
	return 1;
}


int fread_vec_ascii ( char *filename, Vector *new_vector ) {
	FILE *file = fopen( filename, "r" );
	if ( file == NULL ) {
		printf( "Problem reading file %s!\n", filename );
		return -1;
	}

	char line[500]; /* or other suitable maximum line size */
	int numRows = 0;
	while ( fgets( line, sizeof line, file ) != NULL ) /* read a line */
	{
		numRows++;
	}
	fseek( file, 0L, SEEK_SET );

	int rowNumber = 1;
	*new_vector = my_vector( 1, numRows );
	while ( fgets( line, sizeof line, file ) != NULL ) /* read a line */
	{
		double tmp;
		sscanf( line, "%lf ", &tmp );
		(*new_vector)[rowNumber] = tmp;
		rowNumber++;
	}
	fclose( file );
	return numRows;
}


int get_mat_nrows ( Matrix matrix ) {
	return (int) matrix[0][NR];
}


int get_mat_ncols ( Matrix matrix ) {
	return (int) matrix[0][NC];
}


int get_vec_nrows ( Vector vector ) {
	return (int) vector[0];
}


void crossProduct ( Vector a, Vector b, Vector c ) {
	c[1] = a[2] * b[3] - a[3] * b[2];
	c[2] = a[3] * b[1] - a[1] * b[3];
	c[3] = a[1] * b[2] - a[2] * b[1];
}


double vec_norm ( Vector a ) {
	return sqrt( sqr(a[1]) + sqr(a[2]) + sqr(a[3]));
}


int vec_to_mat_op ( Vector a, Matrix c, double (*op) (double) ) {

	int a_rows = (int) a[NR];
	int c_rows = (int) c[0][NR];
	int c_cols = (int) c[0][NC];

	int i;


	if ( a_rows == c_rows && c_cols == 1   ) {
		for ( i = 1; i <= a_rows; ++i )
			c[i][1] = op == 0 ? a[i] : op( a[i] );
		return TRUE;
	}
	else if ( a_rows == c_cols && c_rows == 1   ) {
		for ( i = 1; i <= a_rows; ++i )
			c[1][i] = op == 0 ? a[i] : op( a[i] );
		return TRUE;
	}

	return FALSE;

}

void my_basic_free_matrix ( Matrix a ) {

	if ( a != 0) {
		my_free_matrix(a, 1, (int) a[0][NR], 1, (int) a[0][NC] );
	}

}
