#ifndef IAS_MATRIX_UTILITIES_H_
#define IAS_MATRIX_UTILITIES_H_ 1

#include "utility.h"

int fread_mat_ascii(char *filename, Matrix *new_matrix, int ncols);
int fwrite_mat_ascii(char  *filename, Matrix matrix);
int fread_vec_ascii(char *filename, Vector *new_vector);

int malloc_mat_ascii ( char *filename, Matrix *new_matrix );


int get_mat_nrows(Matrix matrix);
int get_mat_ncols(Matrix matrix);

int get_vec_nrows(Vector vector);

void crossProduct(Vector a, Vector b, Vector c);
double vec_norm(Vector a);

/* Transforms a Vector to a Matrix.
 * When transforming, it applies at each element the operation specified in op.
 * If op == 0 then no operation is applied.
 */
int vec_to_mat_op ( Vector a, Matrix c, double (*op) (double) );


/*
 * Used for deleting matrices indexed at (1,1)
 */
void my_basic_free_matrix ( Matrix a );

#endif /* IAS_MATRIX_UTILITIES_H_ */
