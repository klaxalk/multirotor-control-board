/*
 * kalman.h
 *
 *  Author: Tomas Baca
 */ 

#include "kalman.h"
#include "system.h"

kalmanHandler elevatorKalmanHandler;

void kalmanIteration(kalmanHandler handler, const vector_float * measurement, const vector_float * input, const matrix_float * A, const matrix_float * A_transpose, const matrix_float * B, const matrix_float * R, const matrix_float * Q, const matrix_float * C, const matrix_float * C_transpose, const float dt) {

	/* -------------------------------------------------------------------- */
	/*	Copy the kalman variables locally									*/
	/* -------------------------------------------------------------------- */

	kalmanHandler handler_local;
	
	// number of states
	int n = handler.states->length;
	
	// number of inputs
	int u = input->length;
	
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
	float temp_vector_data[n];							// local array for data
	vector_float temp_vector;							// local temp vector of length n
	temp_vector.data = (float *) &temp_vector_data;
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
	/*	Correction step - Kalman Gain										*/
	/* -------------------------------------------------------------------- */
	
	// compute kalman gain
	// K = covariance*C'*((C*covariance*C' + Q)^-1)
	
	// temp matrix for computing the kalman gain
	float temp_matrix3_data[u*n];
	matrix_float temp_matrix3;
	temp_matrix3.data = (float *) &temp_matrix3_data;
	temp_matrix3.height = u;
	temp_matrix3.width = n;
	
	float temp_matrix4_data[u*u];
	matrix_float temp_matrix4;
	temp_matrix4.data = (float *) &temp_matrix4_data;
	temp_matrix4.height = u;
	temp_matrix4.width = n;
	
	// temp_matrix3 = C*covariance
	matrix_float_mul(C, handler_local.covariance, &temp_matrix3);
	
	// temp_matrix4 = temp_matrix3*C'
	matrix_float_mul(&temp_matrix3, C_transpose, &temp_matrix4);
	
	// temp_matrix4 += Q
	matrix_float_add(&temp_matrix4, Q);
	
	// temp_matrix4^-1
	matrix_float_inverse(&temp_matrix4);
	
	// convert temp_matrix3 to n*u
	temp_matrix3.height = n;
	temp_matrix3.width = u;
	
	// temp_matrix3 = covariance*C'
	matrix_float_mul(handler_local.covariance, C_transpose, &temp_matrix3);
	
	// matrix for the kalman gain
	float kalman_gain_data[n*u];
	matrix_float K;
	K.data = (float *) &kalman_gain_data;
	K.height = n;
	K.width = u;
	
	// K = temp_matrix3*temp_matrix4
	matrix_float_mul(&temp_matrix3, &temp_matrix4, &K);
	
	/* -------------------------------------------------------------------- */
	/*	Correction step - Recomputing states								*/
	/* -------------------------------------------------------------------- */
	
	// K = covariance*C'*((C*covariance*C' + Q)^-1);
	
	// temp matrix for C*states
	float temp_vector2_data[u];
	vector_float temp_vector2;
	temp_vector2.data = (float *) &temp_vector2_data;
	temp_vector2.length = u;
	temp_vector2.orientation = 0;
	
	// temp_matrix5 = C*states
	matrix_float_mul_vec_right(C, handler_local.states, &temp_vector2);
	
	// temp_vector2 = measurement - temp_vector2
	vector_float_times(&temp_vector2, (float) -1);
	vector_float_add(&temp_vector2, measurement);
	
	// temp_vector = K*temp_vector2
	matrix_float_mul_vec_right(&K, &temp_vector2, &temp_vector);
	
	// states = states + temp_vector2
	vector_float_add(handler_local.states, &temp_vector);
	
	/* -------------------------------------------------------------------- */
	/*	Correction step - Recomputing covariance							*/
	/* -------------------------------------------------------------------- */
	
	// covariance = (eye(n) - K*C)*covariance;
	
	// temp_matrix = K*C
	matrix_float_mul(&K, C, &temp_matrix);
	
	// eye(n) - temp_matrix
	matrix_float_times(&temp_matrix, (float) -1);
	int i;
	for (i = 1; i <= temp_matrix.height; i++) {
		matrix_float_set(&temp_matrix, i, i, matrix_float_get(&temp_matrix, i, i) + (float) 1);
	}
	
	// temp_matrix2 = temp_matrix*covariance
	matrix_float_mul(&temp_matrix, handler_local.covariance, &temp_matrix2);
	
	/* -------------------------------------------------------------------- */
	/*	Copy output to the handler											*/
	/* -------------------------------------------------------------------- */
	
	portENTER_CRITICAL();
	
	vector_float_copy(handler.states, handler_local.states);
	matrix_float_copy(handler.covariance, &temp_matrix2);
	
	portEXIT_CRITICAL();
}

void kalmanInit(kalmanHandler handler) {
	
	matrix_float_set_identity(handler.covariance);
	vector_float_set_zero(handler.states);
}	