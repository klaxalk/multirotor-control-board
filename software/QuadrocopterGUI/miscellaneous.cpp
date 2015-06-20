/*
 * miscellaneous.h
 *
 *  Author: Tomas Baca
 */

#include "CMatrixLib.h"
#include <stdlib.h>
//#include "system.h"

/**
 * dynamically allocate the matrix using FreeRTOS malloc
 */
matrix_float * matrix_float_alloc(const int16_t h, const int16_t w) {

    matrix_float * m = 0;

    // dimensions must be positive
    if ((h > 0) && (w > 0)) {

        m = (matrix_float *) malloc(sizeof (matrix_float));

        // if didn't failed to allocated the space
        if (m != 0) {

            m->height = h;
            m->width = w;
            m->data = (float *) malloc(w*h*sizeof(float));
        }
    }

    return m;
}

/**
 * dynamically allocate the matrix using FreeRTOS malloc without a data
 */
matrix_float * matrix_float_alloc_hollow(const int16_t h, const int16_t w, float * data_pointer) {

    matrix_float * m = 0;

    // dimensions must be positive
    if ((h > 0) && (w > 0)) {

        m = (matrix_float *) malloc(sizeof (matrix_float));

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

        v = (vector_float *) malloc(sizeof (vector_float));

        // if didn't failed to allocated the space
        if (v != 0) {

            v->length = length;
            v->orientation = orientation;
            v->data = (float *) malloc(length*sizeof(float));
        }
    }

    return v;
}

// vector allocation without a data
vector_float * vector_float_alloc_hollow(const int16_t length, int8_t orientation, float * data_pointer) {

    vector_float * v = 0;

    // dimension must be positive
    if (length > 0) {

        v = (vector_float *) malloc(sizeof (vector_float));

        // if didn't failed to allocated the space
        if (v != 0) {

            v->length = length;
            v->orientation = orientation;
            v->data = data_pointer;
        }
    }

    return v;
}

// deallocate the matrix using FreeRTOS free
void matrix_float_free(matrix_float * m) {

    free(m->data);
    free(m);
}

// deallocate the matrix using FreeRTOS free without a data deallocation
void matrix_float_free_hollow(matrix_float * m) {

    free(m);
}

// vector deallocation
void vector_float_free(vector_float * v) {

    free(v->data);
    free(v);
}

// vector deallocation without a data deallocation
void vector_float_free_hollow(vector_float * v) {

    free(v);
}

// print the matrix to serial output
