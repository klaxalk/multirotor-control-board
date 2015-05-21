/*
 * miscellaneous.h
 *
 *  Author: Tomas Baca
 */

#ifndef MISCELLANEOUS_H_
#define MISCELLANEOUS_H_

//#include "system.h"
#include "CMatrixLib.h"
//#include "system.h"

// dynamically allocate the matrix using FreeRTOS pvPortMalloc
matrix_float * matrix_float_alloc(const int16_t h, const int16_t w);

matrix_float * matrix_float_alloc_hollow(const int16_t h, const int16_t w, float * data_pointer);

// dynamically allocate the vector using FreeRTOS pvPortMalloc
vector_float * vector_float_alloc(const int16_t length, int8_t orientation);

vector_float * vector_float_alloc_hollow(const int16_t length, int8_t orientation, float * data_pointer);

// deallocate the matrix using FreeRTOS vPortFree
void matrix_float_free(matrix_float * m);

void matrix_float_free_hollow(matrix_float * m);

// deallocate the vector using FreeRTOS vPortFree
void vector_float_free(vector_float * v);

void vector_float_free_hollow(vector_float * v);

// print the matrix to serial output
void matrix_float_print(const matrix_float * a);

// print the matrix to serial output
void vector_float_print(const vector_float * a);

#endif // MISCELLANEOUS_H_
