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

	USART_puts(UART4, "Matrix: ");
	USART_puts(UART4, a->name);
	USART_puts(UART4, "\n\r");

	for (i = 1; i <= a->height; i++) {

		for (j = 1; j <= a->width; j++) {

			sprintf(temp, "%12.8f", matrix_float_get(a, i, j));
			USART_puts(UART4, temp);

			if (j < a->width)
			USART_puts(UART4, ", ");

		}

		USART_puts(UART4, "\n\r");
	}
	USART_puts(UART4, "\n\r");
}

// print the matrix to serial output
void vector_float_print(const vector_float * a) {

	int8_t i;
	char temp[40];

	USART_puts(UART4, "Vector: ");
	USART_puts(UART4, a->name);
	USART_puts(UART4, "\n\r");

	for (i = 1; i <= a->length; i++) {

		sprintf(temp, "%8.4f", vector_float_get(a, i));
		USART_puts(UART4, temp);

		if (a->orientation == 1) {
			if (i < a->length) {
				USART_puts(UART4, ", ");
			} else {
				USART_puts(UART4, "\n\r");
			}
		} else {
			USART_puts(UART4, "\n\r");
		}

	}
	USART_puts(UART4, "\n\r");
}
