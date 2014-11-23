/*
 * matrixLib.h
 *
 *  Author: Tomas Baca
 */ 


#ifndef MATRIXLIB_H_
#define MATRIXLIB_H_

#include <stdint.h>

typedef struct {
	
	int16_t width;
	int16_t height;
	float * data;
} matrix_float;

typedef struct {
	
	int16_t length;
	int8_t orientation;
	float * data;
} vector_float;

// allocation
matrix_float * matrix_float_alloc(const int16_t h, const int16_t w);

// deallocation
void matrix_float_free(matrix_float * m);

// set particular cell of the matrix
void matrix_float_set(matrix_float * m, const int16_t h, const int16_t w, const float value);

// matrix transposition
int matrix_float_transpose(matrix_float * m);

// set matrix to zeros
int matrix_float_set_zero(matrix_float * m);

// set all cells to value
int matrix_float_set_all(matrix_float * m, const float value);

// set matrix to identity matrix
int matrix_float_set_identity(matrix_float * m);

// add two matrices
int matrix_float_add(matrix_float * a, const matrix_float *b);

// subtract two matrices
int matrix_float_sub(matrix_float * a, const matrix_float *b);

// multiply two matrices
int matrix_float_mul(const matrix_float * a, const matrix_float * b, matrix_float * C);

#endif /* MATRIXLIB_H_ */