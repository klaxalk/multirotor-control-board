/*
 * matrixLib.c
 *
 *  Author: Tomas Baca
 */ 

#include "matrixLib.h"
#include "FreeRTOS.h"

// dynamically allocate the matrix using FreeRTOS pvPortMalloc
matrix_float * matrix_float_alloc(const int16_t h, const int16_t w) {
	
	matrix_float * m;
	
	m = 0;
	
	// dimensions must be positive
	if ((h > 0) && (w > 0)) {
		
		m = (matrix_float *) pvPortMalloc(sizeof (matrix_float));
		
		// if failed to allocated the space
		if (m != 0) {
			
			m->height = h;
			m->width = w;
			m->data = (float *) pvPortMalloc(w*h*sizeof(float));
		}
	}
	
	return m;
}

// deallocate the matrix using FreeRTOS vPortFree
void matrix_float_free(matrix_float * m) {
	
	vPortFree(m->data);
	vPortFree(m);
}

// set particular cell of the matrix
void matrix_float_set(matrix_float * m, const int16_t h, const int16_t w, const float value) {
	
	m->data[(w-1)*m->height + h - 1] = value;
}

// get the particular cell of the matrix
float matrix_float_get(const matrix_float * m, const int16_t h, const int16_t w) {
	
	return m->data[(w-1)*m->height + h - 1];
}

// matrix transposition
void matrix_float_transpose(matrix_float * m) {
	
	int16_t i, j;
	float temp;
	
	for (i = 1; i <= m->height; i++) {
		
		for (j = 1; j <= m->width; j++) {
			
			temp = matrix_float_get(m, i, j);
			matrix_float_set(m, i, j, matrix_float_get(m, j, i));
			matrix_float_set(m, j, i, temp);
		}
	}
}


// set all cells to value
void matrix_float_set_all(matrix_float * m, const float value) {
	
	int16_t i, j;
	
	for (i = 1; i <= m->height; i++) {
		
		for (j = 1; j <= m->width; j++) {
			
			matrix_float_set(m, i, j, value);
		}
	}
}

// set matrix to zeros
void matrix_float_set_zero(matrix_float * m) {
	
	matrix_float_set_all(m, (float) 0);
}

// set matrix to identity matrix
void matrix_float_set_identity(matrix_float * m) {
	
	// matrix must be a square matrix
	if (m->width == m->height) {
	
		matrix_float_set_zero(m);
		
		int16_t i;
		
		for (i = 1; i <= m->height; i++) {
			
			matrix_float_set(m, i, i, (float) 1);	
		}
	}
}

// add two matrices
void matrix_float_add(matrix_float * a, const matrix_float *b) {
	
	// matrices dimensions must agree
	if ((a->width == b->width) && (a->height == b->height)) {
		
		int16_t i, j;
	
		for (i = 1; i <= a->height; i++) {
		
			for (j = 1; j <= a->width; j++) {
			
				matrix_float_set(a, i, j, matrix_float_get(a, i, j) + matrix_float_get((matrix_float *) b, i, j));
			}
		}
	}
}


// subtract two matrices
void matrix_float_sub(matrix_float * a, const matrix_float * b) {
	
	// matrices dimensions must agree
	if ((a->width == b->width) && (a->height == b->height)) {
		
		int16_t i, j;
		
		for (i = 1; i <= a->height; i++) {
			
			for (j = 1; j <= a->width; j++) {
				
				matrix_float_set(a, i, j, matrix_float_get(a, i, j) - matrix_float_get((matrix_float *) b, i, j));
			}
		}
	}			
}

// set the particular cell of the vector
void vector_float_set(vector_float * v, const int16_t pos, const float value) {
	
	//! TOTO implement
}

// set the particular cell of the vector
float vector_float_get(vector_float * v, const int16_t pos) {
	
	//! TODO implement
	return 0;
}

// get row from matrix
void matrix_float_get_row(const matrix_float * m, vector_float * v, const int16_t row) {
	
	// the matrix width must be same as the vector length
	if ((v->length == m->width)) {
		
		int16_t i;
		
		for (i = 1; i <= m->width; i++) {
			
			vector_float_set(v, i, matrix_float_get(m, row, i));
		}
		
		v->orientation = 1;
	}
}

// get column from matrix
void matrix_float_get_col(const matrix_float * m, vector_float * v, const int16_t col) {
	
	// the matrix width must be same as the vector length
	if ((v->length == m->height)) {
		
		int16_t i;
		
		for (i = 1; i <= m->height; i++) {
			
			vector_float_set(v, i, matrix_float_get(m, i, col));
		}
		
		v->orientation = 0;
	}
}

// multiply two matrices
void matrix_float_mul(const matrix_float * a, const matrix_float * b, matrix_float * C) {
	
	//! TODO implement
}

