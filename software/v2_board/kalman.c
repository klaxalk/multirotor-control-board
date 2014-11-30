/*
 * kalman.h
 *
 *  Author: Tomas Baca
 */ 

#include "kalman.h";

void kalmanIteration(matrix_float * states, matrix_float * covariance, const matrix_float * measurement, const matrix_float * u, const matrix_float * A, const matrix_float * B, const matrix_float * Q, const matrix_float * C, const float dt) {

	/* -------------------------------------------------------------------- */
	/*	Prediction step														*/
	/* -------------------------------------------------------------------- */
	
	matrix_float tempStates;
	
	/* -------------------------------------------------------------------- */
	/*	Correction step														*/
	/* -------------------------------------------------------------------- */
}

void kalmanInit() {
	
}