/*
 * miscellaneous.h
 *
 *  Author: Tomas Baca
 */

#include "CMatrixLib.h"
#include "system.h"

// print the matrix to serial output
void matrix_float_print(const matrix_float * a) {

	int8_t i, j;
	char temp[50];

	Usart4PutString("Matrix: ");
	Usart4PutString(a->name);
	Usart4PutString("\n\r");

	for (i = 1; i <= a->height; i++) {

		for (j = 1; j <= a->width; j++) {

			sprintf(temp, "%12.8f", matrix_float_get(a, i, j));
			Usart4PutString(temp);

			if (j < a->width)
			Usart4PutString(", ");

		}

		Usart4PutString("\n\r");
	}
	Usart4PutString("\n\r");
}

// print the matrix to serial output
void vector_float_print(const vector_float * a) {

	int8_t i;
	char temp[40];

	Usart4PutString("Vector: ");
	Usart4PutString(a->name);
	Usart4PutString("\n\r");

	for (i = 1; i <= a->length; i++) {

		sprintf(temp, "%8.4f", vector_float_get(a, i));
		Usart4PutString(temp);

		if (a->orientation == 1) {
			if (i < a->length) {
				Usart4PutString(", ");
			} else {
				Usart4PutString("\n\r");
			}
		} else {
			Usart4PutString("\n\r");
		}

	}
	Usart4PutString("\n\r");
}
