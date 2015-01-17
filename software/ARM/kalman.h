/*
 * kalman.h
 *
 *  Author: Tomas Baca
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "CMatrixLib.h"

typedef struct {

	matrix_float * covariance;
	vector_float * states;
	int8_t number_of_states;
	int8_t number_of_inputs;
	matrix_float * temp_matrix_n_n;
	matrix_float * temp_matrix2_n_n;
	matrix_float * temp_matrix3_u_n;
	matrix_float * temp_matrix4_u_u;
	vector_float * temp_vector_n;
	vector_float * temp_vector_u;
} kalmanHandler;

void kalmanIteration(kalmanHandler * handler, const vector_float * measurement, const vector_float * input, const matrix_float * A, const matrix_float * B, const matrix_float * R, const matrix_float * Q, const matrix_float * C, const float dt);

void kalmanInit(kalmanHandler * handler);

#endif /* KALMAN_H_ */
