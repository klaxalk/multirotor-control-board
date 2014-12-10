/*
 * kalman.h
 *
 *  Author: Tomas Baca
 */ 

#ifndef KALMAN_H_
#define KALMAN_H_

#include "matrixLib.h"

typedef struct {
	
	matrix_float * covariance;
	vector_float * states;
} kalmanHandler;

void kalmanIteration(kalmanHandler handler, const vector_float * measurement, const vector_float * input, const matrix_float * A, const matrix_float * A_transpose, const vector_float * B, const matrix_float * R, const matrix_float * Q, const matrix_float * C, const matrix_float * C_transpose, const float dt);

void kalmanInit(kalmanHandler handler);

#endif /* KALMAN_H_ */