/*
 * kalman.h
 *
 *  Author: Tomas Baca
 */

#include "kalman.h"
#include "system.h"
#include "CMatrixLib.h"

void kalmanIteration(kalmanHandler * handler, const vector_float * measurement, const vector_float * input, const matrix_float * A, const matrix_float * B, const matrix_float * R, const matrix_float * Q, const matrix_float * C, const float dt) {

	/* -------------------------------------------------------------------- */
	/*	Copy the kalman variables locally									*/
	/* -------------------------------------------------------------------- */

	kalmanHandler handler_local;

	// copy of the states vector
	vector_float handler_local_states;
	handler_local_states.name = "Local states";
	float handler_local_states_data[handler->number_of_states];
	handler_local_states.data = (float *) &handler_local_states_data;
	handler_local_states.length = handler->number_of_states;

	handler_local.states = &handler_local_states;
	vector_float_copy(handler_local.states, handler->states);

	// copy of the covariance matrix
	matrix_float handler_local_covariance;
	handler_local_covariance.name = "Local covariance";
	handler_local_covariance.height = handler->covariance->height;
	handler_local_covariance.width = handler->covariance->width;
	float handler_local_covariance_data[handler->number_of_states*handler->number_of_states];
	handler_local_covariance.data = (float *) &handler_local_covariance_data;
	handler_local.covariance = &handler_local_covariance;

	handler_local.covariance = &handler_local_covariance;
	matrix_float_copy(handler_local.covariance, handler->covariance);

	/* -------------------------------------------------------------------- */
	/*	prediction step														*/
	/* -------------------------------------------------------------------- */

	// recompute new states

	// temp_states = a*handler->states;
	matrix_float_mul_vec_right(A, handler_local.states, handler->temp_vector_n);

	vector_float_copy(handler_local.states, handler->temp_vector_n);

	// temp_states = temp_states + b*input
	matrix_float_mul_vec_right(B, input, handler->temp_vector_n);

	vector_float_add(handler_local.states, handler->temp_vector_n);

	// recompute covariance

	// temp_matrix = A*covariance
	matrix_float_mul(A, handler_local.covariance, handler->temp_matrix_n_n);

	// temp_matrix2 = temp_matrix*A'
	matrix_float_mul_trans(handler->temp_matrix_n_n, A, handler->temp_matrix2_n_n);

	// covariance = temp_matrix2 + R
	matrix_float_copy(handler_local.covariance, handler->temp_matrix2_n_n);
	matrix_float_add(handler_local.covariance, R);

	/* -------------------------------------------------------------------- */
	/*	Correction step - Kalman Gain										*/
	/* -------------------------------------------------------------------- */

	// compute kalman gain
	// K = covariance*C'*((C*covariance*C' + Q)^-1)

	// temp_matrix3 = C*covariance
	matrix_float_mul(C, handler_local.covariance, handler->temp_matrix3_u_n);

	// temp_matrix4 = temp_matrix3*C'
	matrix_float_mul_trans(handler->temp_matrix3_u_n, C, handler->temp_matrix4_u_u);

	// temp_matrix4 += Q
	matrix_float_add(handler->temp_matrix4_u_u, Q);

	// temp_matrix4^-1
	matrix_float_inverse(handler->temp_matrix4_u_u);

	// convert temp_matrix3 to n*u
	handler->temp_matrix3_u_n->height = handler->number_of_states;
	handler->temp_matrix3_u_n->width = handler->number_of_inputs;

	// temp_matrix3 = covariance*C'
	matrix_float_mul_trans(handler_local.covariance, C, handler->temp_matrix3_u_n);

	// matrix for the kalman gain
	matrix_float K;
	K.name = "Kalman Gain";
	float kalman_gain_data[handler->number_of_inputs*handler->number_of_states];
	K.data = (float *) &kalman_gain_data;
	K.height = handler->number_of_states;
	K.width = handler->number_of_inputs;

	// K = temp_matrix3*temp_matrix4
	matrix_float_mul(handler->temp_matrix3_u_n, handler->temp_matrix4_u_u, &K);

	// convert temp_matrix3 to original dimensions
	handler->temp_matrix3_u_n->height = handler->number_of_inputs;
	handler->temp_matrix3_u_n->width = handler->number_of_states;

	/* -------------------------------------------------------------------- */
	/*	Correction step - Recomputing states								*/
	/* -------------------------------------------------------------------- */

	// K = covariance*C'*((C*covariance*C' + Q)^-1);

	// temp_matrix2 = C*states
	matrix_float_mul_vec_right(C, handler_local.states, handler->temp_vector_u);

	// temp_vector2 = measurement - temp_vector2
	vector_float_times(handler->temp_vector_u, (float) -1);

	vector_float_add(handler->temp_vector_u, measurement);

	// temp_vector = K*tem p_vector2
	matrix_float_mul_vec_right(&K, handler->temp_vector_u, handler->temp_vector_n);

	// states = states + temp_vector2
	vector_float_add(handler_local.states, handler->temp_vector_n);

	/* -------------------------------------------------------------------- */
	/*	Correction step - Recomputing covariance						    */
	/* -------------------------------------------------------------------- */

	// covariance = (eye(n) - K*C)*covariance;

	// temp_matrix = K*C
	matrix_float_mul(&K, C, handler->temp_matrix_n_n);

	// eye(n) - temp_matrix
	matrix_float_times(handler->temp_matrix_n_n, (float) -1);
	int i;
	for (i = 1; i <= handler->temp_matrix_n_n->height; i++) {
		matrix_float_set(handler->temp_matrix_n_n, i, i, matrix_float_get(handler->temp_matrix_n_n, i, i) + (float) 1);
	}

	// temp_matrix2 = temp_matrix*covariance
	matrix_float_mul(handler->temp_matrix_n_n, handler_local.covariance, handler->temp_matrix2_n_n);

	/* -------------------------------------------------------------------- */
	/*	Copy output to the handler											*/
	/* -------------------------------------------------------------------- */

	portENTER_CRITICAL();

	vector_float_copy(handler->states, handler_local.states);
	matrix_float_copy(handler->covariance, handler->temp_matrix2_n_n);

	portEXIT_CRITICAL();
}

void kalmanInit(kalmanHandler * handler) {

	matrix_float_set_identity(handler->covariance);

	// p.p.
	vector_float_set(handler->states, 1, 0);
	vector_float_set(handler->states, 2, 0);
	vector_float_set(handler->states, 3, 0);
}

