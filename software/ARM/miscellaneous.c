/*
 * miscellaneous.h
 *
 *  Author: Tomas Baca
 */

#include "CMatrixLib.h"
#include "system.h"

/**
 * dynamically allocate the matrix using FreeRTOS pvPortMalloc
 */
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

/**
 * dynamically allocate the matrix using FreeRTOS pvPortMalloc without a data
 */
matrix_float * matrix_float_alloc_hollow(const int16_t h, const int16_t w, float * data_pointer) {

	matrix_float * m = 0;

	// dimensions must be positive
	if ((h > 0) && (w > 0)) {

		m = (matrix_float *) pvPortMalloc(sizeof (matrix_float));

		// if didn't failed to allocated the space
		if (m != 0) {

			m->height = h;
			m->width = w;
			m->data = data_pointer;
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

// vector allocation without a data
vector_float * vector_float_alloc_hollow(const int16_t length, int8_t orientation, float * data_pointer) {

	vector_float * v = 0;

	// dimension must be positive
	if (length > 0) {

		v = (vector_float *) pvPortMalloc(sizeof (vector_float));

		// if didn't failed to allocated the space
		if (v != 0) {

			v->length = length;
			v->orientation = orientation;
			v->data = data_pointer;
		}
	}

	return v;
}

// deallocate the matrix using FreeRTOS vPortFree
void matrix_float_free(matrix_float * m) {

	vPortFree(m->data);
	vPortFree(m);
}

// deallocate the matrix using FreeRTOS vPortFree without a data deallocation
void matrix_float_free_hollow(matrix_float * m) {

	vPortFree(m);
}

// vector deallocation
void vector_float_free(vector_float * v) {

	vPortFree(v->data);
	vPortFree(v);
}

// vector deallocation without a data deallocation
void vector_float_free_hollow(vector_float * v) {

	vPortFree(v);
}

// print the matrix to serial output
void matrix_float_print(const matrix_float * a) {

	int_least16_t i, j;
	char temp[50];

	usart4PutString("Matrix: ");
	usart4PutString(a->name);
	usart4PutString("\n\r");

	for (i = 1; i <= a->height; i++) {

		for (j = 1; j <= a->width; j++) {

			sprintf(temp, "%6.2f", matrix_float_get(a, i, j));
			usart4PutString(temp);

			if (j < a->width)
			usart4PutString(", ");

		}

		vTaskDelay(5);

		usart4PutString("\n\r");
	}
	usart4PutString("\n\r");
}

// print the matrix to serial output
void vector_float_print(const vector_float * a) {

	int_least16_t i;
	char temp[60];

	usart4PutString("Vector: ");
	usart4PutString(a->name);
	usart4PutString("\n\r");

	for (i = 1; i <= a->length; i++) {

		sprintf(temp, "%12.6f", vector_float_get(a, i));
		usart4PutString(temp);

		if (a->orientation == 1) {
			if (i < a->length) {
				usart4PutString(", ");
			} else {
				usart4PutString("\n\r");
			}
		} else {
			usart4PutString("\n\r");
		}

		vTaskDelay(10);
	}
	usart4PutString("\n\r");
}
