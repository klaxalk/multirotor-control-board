/*
 * matrixLib.c
 *
 *  Author: Tomas Baca
 */ 

#include "matrixLib.h"
#include "FreeRTOS.h"
#include <stdio.h>

// dynamically allocate the matrix using FreeRTOS pvPortMalloc
matrix_float * matrix_float_alloc(const int16_t h, const int16_t w) {
	
	matrix_float * m = 0;
	
	// dimensions must be positive
	if ((h > 0) && (w > 0)) {
		
		m = (matrix_float *) pvPortMalloc(sizeof (matrix_float));
		
		// if didn't failed to allocated the space
		if (m != 0) {
			
			m->height = h;
			m->width = w;
			m->data = (float *) pvPortMalloc(w*h*sizeof(float));
		}
	}
	
	return m;
}

// vector allocation
vector_float * vector_float_alloc(const int16_t length, int8_t orientation) {
	
	vector_float * v = 0;
	
	// dimension must be positive
	if (length > 0) {
		
		v = (vector_float *) pvPortMalloc(sizeof (vector_float));
		
		// if didn't failed to allocated the space
		if (v != 0) {
			
			v->length = length;
			v->orientation = orientation;
			v->data = (float *) pvPortMalloc(length*sizeof(float));
		}	
	}
	
	return v;
}

// deallocate the matrix using FreeRTOS vPortFree
void matrix_float_free(matrix_float * m) {
	
	vPortFree(m->data);
	vPortFree(m);
}

// vector deallocation
void vector_float_free(vector_float * v) {
	
	vPortFree(v->data);
	vPortFree(v);
}

// add two vector a, b and write the answer to a
void vector_float_add(vector_float * a, const vector_float * b) {
	
	// check dimension
	if (a->length == b->length) {
	
		int16_t i;
	
		for (i = 1; i <= a->length; i++) {
			
			vector_float_set(a, i, vector_float_get(a, i) + vector_float_get(b, i));
		}
	}
}

// multiplies a vector by a constant
void vector_float_times(vector_float * a, const float C) {
	
	int16_t i;
	
	for (i = 1; i <= a->length; i++) {
			
		vector_float_set(a, i, vector_float_get(a, i)*C);
	}
}

// copy vector b to a vector a
void vector_float_copy(vector_float * a, const vector_float * b) {
	
	// check dimension
	if (a->length == b->length) {
		
		memcpy(a->data, b->data, a->length*sizeof(float));
		a->orientation = b->orientation;
	}
}

// set particular cell of the matrix
void matrix_float_set(matrix_float * m, const int16_t h, const int16_t w, const float value) {
	
	m->data[(h-1)*m->width + w - 1] = value;
}

// set the whole vector to zeros
void vector_float_set_zero(vector_float * v) {
	
	int16_t i;
	
	for (i = 1; i <= v->length; i++) {
		
		vector_float_set(v, i, 0);
	}
}

// get the particular cell of the matrix
float matrix_float_get(const matrix_float * m, const int16_t h, const int16_t w) {
	
	return m->data[(h-1)*m->width + w - 1];
}

// matrix transposition
void matrix_float_transpose(matrix_float * m) {
	
	int16_t i, j;
	float temp;
	
	for (i = 1; i <= m->height; i++) {
		
		for (j = i; j <= m->width; j++) {
			
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
void matrix_float_copy(matrix_float * a, const matrix_float * b) {
	
	// matrices dimensions must agree
	if ((a->width == b->width) && (a->height == b->height)) {
		
		memcpy(a->data, b->data, a->height*a->width*sizeof(float));
	}
}

// add two matrices
void matrix_float_add(matrix_float * a, const matrix_float * b) {
	
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
	
	if (pos >= 1 && pos <= v->length) {
	
		v->data[pos-1] = value;	
	} else {
		
		// index out of bounds
	}
}

// set the particular cell of the vector
float vector_float_get(const vector_float * v, const int16_t pos) {
	
	if (pos >= 1 && pos <= v->length) {
	
		return v->data[pos-1];	
	}
	
	// else index out of bounds
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
		
		// set orientation to horizontal
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
		
		// set orientation to vertical
		v->orientation = 0;
	}
}

// multiply two matrices
// the naive way
void matrix_float_mul(const matrix_float * a, const matrix_float * b, matrix_float * C) {
	
	int16_t i, j, k;
	float tempSum;
	
	// dimensions must agree
	if (a->width == b->height && a->height == C->height && b->width == C->width) {
		
		for (i = 1; i <= C->height; i++) {
			
			for (j = 1; j <= C->width; j++) {
				
				tempSum = 0;
				for (k = 1; k <= a->width; k++) {
					tempSum += matrix_float_get(a, i, k)*matrix_float_get(b, k, j);
				}
				
				matrix_float_set(C, i, j, tempSum);
			}
		}
	} 
}

// multiply matrix by a constant
void matrix_float_times(matrix_float * a, const float C) {
	
	int16_t i, j;
	
	for (i = 1; i <= a->height; i++) {
		
		for (j = 1; j <= a->width; j++) {
			
			matrix_float_set(a, i, j, C*matrix_float_get(a, i, j));
		}
	}
}

// multiply a matrix by a vector from the right
void matrix_float_mul_vec_right(const matrix_float * m, const vector_float * v, vector_float * C) {
	
	int16_t i, j;
	float tempSum;
	
	// dimensions must agree
	if (m->width == v->length) {
		
		for (i = 1; i <= m->height; i++) {
			
			tempSum = 0;
			
			for (j = 1; j <= v->length; j++) {
			
				tempSum += matrix_float_get(m, i, j)*vector_float_get(v, j);	
			}
			vector_float_set(C, j, tempSum);
		}
	}	
	
	// set orientation to vertical
	C->orientation = 0;
}

// multiply a matrix by a vector from the left
void matrix_float_mul_vec_left(const matrix_float * m, const vector_float * v, vector_float * C) {
	
	int16_t i, j;
	float tempSum;
	
	// dimensions must agree
	if (m->height == v->length) {
		
		for (i = 1; i <= m->height; i++) {
			
			tempSum = 0;
			
			for (j = 1; j <= v->length; j++) {
				
				tempSum += matrix_float_get(m, j, i)*vector_float_get(v, j);
			}
			vector_float_set(C, j, tempSum);
		}
	}
	
	// set orientation to horizontal
	C->orientation = 1;
}

// compute the determinant of a matrix
float matrix_float_determinant(const matrix_float * a) {
	
	// the matrix should be a square matrix
	if (a->width == a->height) {
		
		// special case, 1x1 matrix
		if (a->width == 1) {
			
			matrix_float_get(a, 1, 1);
		}
	
		int16_t i, j, k;
		float coeficient;
		float determinant = 1;

		// copy matrix a to a local temporary matrix
		int16_t n = a->height;

		float temp_matrix_data[n*n];
		matrix_float temp_matrix;
		temp_matrix.data = (float *) &temp_matrix_data;
		temp_matrix.height = n;
		temp_matrix.width = n;
		temp_matrix.name = "Vypocet determinantu";
	
		// copy the specified matrix into temp_matrix
		matrix_float_copy(&temp_matrix, a);
	
		// start gauss elimination
		// for all rows
		for (i = 1; i <= a->height; i++) {
			
			// for all rows bellow it
			for (j = a->height; j > i; j--) {
				
				coeficient = matrix_float_get(&temp_matrix, j, i)/matrix_float_get(&temp_matrix, i, i);
				
				for (k = 1; k <= a->width; k++) {
				
					matrix_float_set(&temp_matrix, j, k, matrix_float_get(&temp_matrix, j, k) - coeficient*matrix_float_get(&temp_matrix, i, k));	
				}
			}
		}
	
		for (i = 1; i <= a->width; i++) {
			
			determinant *= matrix_float_get(&temp_matrix, i, i);
		}
		
		return determinant;
	}
	
	return (float) 0;
}

#if USED_MCU == XMEGA

// print the matrix to serial output
void matrix_float_print(const matrix_float * a, UsartBuffer * usartBuffer) {
	
	int8_t i, j;
	char temp[20];
	
	usartBufferPutString(usartBuffer, "Matrix: ", 10);
	usartBufferPutString(usartBuffer, a->name, 10);
	usartBufferPutString(usartBuffer, "\n\r", 10);
	
	for (i = 1; i <= a->height; i++) {
		
		for (j = 1; j <= a->width; j++) {
			
			sprintf(temp, "%6.4f", matrix_float_get(a, i, j));
			usartBufferPutString(usartBuffer, temp, 10);
			
			if (j < a->width)
			usartBufferPutString(usartBuffer, ", ", 10);
			
		}
		
		usartBufferPutString(usartBuffer, "\n\r", 10);
	}
	usartBufferPutString(usartBuffer, "\n\r", 10);
}

#endif

// computer the inversion of matrix A, returns 0 if the inversion doesn't exist, 1 otherwise
int matrix_float_inverse(matrix_float * a) {
	
	// the matrix should be a square matrix
	if (a->width == a->height) {
	
		// special case, 1x1 matrix
		if (a->width == 1) {
			
			return 1/matrix_float_get(a, 1, 1);
		}	
		
		float determinant = matrix_float_determinant(a);
		
		char temp[20];
		sprintf(temp, "%8.4f", determinant);
		usartBufferPutString(usart_buffer_4, temp, 10);
		
		// check the matrix regularity
		if (fabs(determinant) >= 0.000000000000000000001) {
			
			int16_t i, j, k;
			float coeficient;
	
			// copy matrix a to a local temporary matrix
			int16_t n = a->height;

			float temp_matrix_data[n*n];
			matrix_float temp_matrix;
			temp_matrix.data = (float *) &temp_matrix_data;
			temp_matrix.height = n;
			temp_matrix.width = n;
			temp_matrix.name = "Puvodni";
			
			// temp matrix2 for computing the covariance matrix
			float temp_matrix2_data[n*n];
			matrix_float temp_matrix2;
			temp_matrix2.data = (float *) &temp_matrix2_data;
			temp_matrix2.height = n;
			temp_matrix2.width = n;
			temp_matrix2.name = "Nova";
			
			// set one matrix as the specified matrix
			matrix_float_copy(&temp_matrix, a);
			
			// set the other as diagonal
			matrix_float_set_identity(&temp_matrix2);
			
			// do complete gauss elimination
			
			// start elimination the bottom triangular matrix
			// for all rows
			for (i = 1; i <= n; i++) {
						
				// for all rows bellow it
				for (j = n; j > i; j--) {
							
					coeficient = matrix_float_get(&temp_matrix, j, i)/matrix_float_get(&temp_matrix, i, i);
							
					for (k = 1; k <= a->width; k++) {
								
						matrix_float_set(&temp_matrix, j, k, matrix_float_get(&temp_matrix, j, k) - coeficient*matrix_float_get(&temp_matrix, i, k));
						matrix_float_set(&temp_matrix2, j, k, matrix_float_get(&temp_matrix2, j, k) - coeficient*matrix_float_get(&temp_matrix2, i, k));
					}
				}
			}
			
			// make 1 in bottom right corner
			
			coeficient = matrix_float_get(&temp_matrix, n, n);
			for (k = 1; k <= a->width; k++) {
									
				matrix_float_set(&temp_matrix2, n, k, matrix_float_get(&temp_matrix2, n, k)/coeficient);
			}
			matrix_float_set(&temp_matrix, n, n, (float) 1);
			
			// start elimination the top triangular matrix
			// for all rows
			for (i = n; i >= 1; i--) {
				
				// for all rows bellow it
				for (j = 1; j < i; j++) {
					
					coeficient = matrix_float_get(&temp_matrix, j, i)/matrix_float_get(&temp_matrix, i, i);
					
					for (k = 1; k <= a->width; k++) {
						
						matrix_float_set(&temp_matrix, j, k, matrix_float_get(&temp_matrix, j, k) - coeficient*matrix_float_get(&temp_matrix, i, k));
						matrix_float_set(&temp_matrix2, j, k, matrix_float_get(&temp_matrix2, j, k) - coeficient*matrix_float_get(&temp_matrix2, i, k));
					}
				}
			}
			
			matrix_float_copy(a, &temp_matrix2);
			
		// matrix does not have its inversion
		} else {
			
			return 0;
		}
	}
	
	return 1;
}