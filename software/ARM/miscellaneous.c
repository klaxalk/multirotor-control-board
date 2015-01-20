/*
 * miscellaneous.h
 *
 *  Author: Tomas Baca
 */

#include "CMatrixLib.h"
#include "system.h"

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
