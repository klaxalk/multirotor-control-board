/*
 * kalman.h
 *
 *  Author: Tomas Baca
 */ 

#include "kalman.h"

kalmanHandler elevatorKalmanHandler;

void kalmanIteration(kalmanHandler handler, const matrix_float * measurement, const vector_float * input, const matrix_float * A, const matrix_float * A_transpose, const matrix_float * B, const matrix_float * R, const matrix_float * Q, const matrix_float * C, const float dt) {

	/* -------------------------------------------------------------------- */
	/*	Copy the kalman variables locally									*/
	/* -------------------------------------------------------------------- */

	kalmanHandler handler_local;
	
	// number of states
	int n = handler.states->length;
	
	// copy of the states vector
	float handler_local_states_data[n];
	vector_float handler_local_states;
	handler_local_states.data = (float *) &handler_local_states_data;
	handler_local_states.length = n;
	
	handler_local.states = &handler_local_states;
	vector_float_copy(handler_local.states, handler.states);

	// copy of the covariance matrix
	float handler_local_covariance_data[n*n];
	matrix_float handler_local_covariance;
	handler_local_covariance.data = (float *) &handler_local_covariance_data;
	handler_local.covariance = &handler_local_covariance;
	matrix_float_copy(handler_local.covariance, handler.covariance);

	/* -------------------------------------------------------------------- */
	/*	Support variables													*/
	/* -------------------------------------------------------------------- */	

	// temp vector for computing the state vector
	float temp_vector_dara[n];							// local array for data
	vector_float temp_vector;							// local temp vector of length n
	temp_vector.data = (float *) &temp_vector_dara;
	temp_vector.length = n;
	
	// temp matrix for computing the covariance matrix
	float temp_matrix_data[n*n];
	matrix_float temp_matrix;
	temp_matrix.data = (float *) &temp_matrix_data;
	temp_matrix.height = n;
	temp_matrix.width = n;
	
	// temp matrix2 for computing the covariance matrix
	float temp_matrix2_data[n*n];
	matrix_float temp_matrix2;
	temp_matrix2.data = (float *) &temp_matrix2_data;
	temp_matrix2.height = n;
	temp_matrix2.width = n;

	/* -------------------------------------------------------------------- */
	/*	Prediction step														*/
	/* -------------------------------------------------------------------- */
	
	// recompute new states
	
	// temp_states = A*handler.states;
	matrix_float_mul_vec_right(A, handler_local.states, &temp_vector);
	vector_float_copy(handler_local.states, &temp_vector);
	
	// temp_states = temp_states + B*input
	matrix_float_mul_vec_right(B, input, &temp_vector);
	vector_float_add(handler_local.states, &temp_vector);
	
	// recompute covariance
	
	// temp_matrix = A*covariance
	matrix_float_mul(A, handler_local.covariance, &temp_matrix);
	
	// temp_matrix2 = temp_matrix*A'
	matrix_float_mul(&temp_matrix, A_transpose, &temp_matrix2);
	
	// covariance = temp_matrix2 + R
	matrix_float_copy(handler_local.covariance, &temp_matrix2);
	matrix_float_add(handler_local.covariance, R);
	
	/* -------------------------------------------------------------------- */
	/*	Correction step														*/
	/* -------------------------------------------------------------------- */
	
	// compute kalman gain
	
	
	
	/* -------------------------------------------------------------------- */
	/*	Copy local handler to the global									*/
	/* -------------------------------------------------------------------- */
	
	vector_float_copy(handler.states, handler_local.states);
	matrix_float_copy(handler.covariance, handler_local.covariance);
}

void kalmanInit(kalmanHandler handler) {
	
	matrix_float_set_identity(handler.covariance);
	vector_float_set_zero(handler.states);
}	