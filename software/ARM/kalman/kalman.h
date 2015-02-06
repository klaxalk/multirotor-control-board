/*
 * kalman.h
 *
 *  Author: Tomas Baca
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "CMatrixLib.h"

typedef struct {

	matrix_float * covariance;	// covariance matrix of kalman system
	vector_float * states;		// states of kalman system
	vector_float * input;		// the input to the system
	matrix_float * system_A;	// main system matrix
	matrix_float * system_B;	// input system matrix
	matrix_float * R_matrix;	// process noise
	vector_float * measurement;	// the measurement vector
	matrix_float * Q_matrix; 	// covariance of the measurement
	matrix_float * C_matrix;	// transfere from measurements to states
	int number_of_states;
	int number_of_inputs;

} kalmanHandler_t;

void kalmanIteration(kalmanHandler_t * kalmanHandler);

#endif /* KALMAN_H_ */
