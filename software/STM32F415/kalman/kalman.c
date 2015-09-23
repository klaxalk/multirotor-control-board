/*
 * kalman.h
 *
 *  Author: Tomas Baca
 */

#include "kalman.h"
#include "system.h"
#include "CMatrixLib.h"

void kalmanIteration(kalmanHandler_t * handler) {

	/* -------------------------------------------------------------------- */
	/*	Copy the kalman variables locally									*/
	/* -------------------------------------------------------------------- */

	kalmanHandler_t handler_local;

	// copy of the states vector
	vector_float handler_local_states;
	float handler_local_states_data[handler->number_of_states];
	handler_local_states.data = (float *) &handler_local_states_data;
	handler_local_states.length = handler->number_of_states;

	handler_local.states = &handler_local_states;
	vector_float_copy(handler_local.states, handler->states);

	// copy of the covariance matrix
	matrix_float handler_local_covariance;
	handler_local_covariance.height = handler->covariance->height;
	handler_local_covariance.width = handler->covariance->width;
	float handler_local_covariance_data[handler->number_of_states*handler->number_of_states];
	handler_local_covariance.data = (float *) &handler_local_covariance_data;

	handler_local.covariance = &handler_local_covariance;
	matrix_float_copy(handler_local.covariance, handler->covariance);

	/* -------------------------------------------------------------------- */
	/* Aux matrices and vectors for part-results							*/
	/* -------------------------------------------------------------------- */

	// temp vector
	vector_float temp_vector_n;
	float temp_vector_data[handler->number_of_states];
	temp_vector_n.data = (float *) &temp_vector_data;
	temp_vector_n.length = handler->number_of_states;
	temp_vector_n.orientation = 0;

	// temp vector2
	vector_float temp_vector2_u;
	float temp_vector2_data[handler->number_of_inputs];
	temp_vector2_u.data = (float *) &temp_vector2_data;
	temp_vector2_u.length = handler->number_of_inputs;
	temp_vector2_u.orientation = 0;

	// temp matrix
	matrix_float temp_matrix_n_n;
	float temp_matrix_data[handler->number_of_states*handler->number_of_states];
	temp_matrix_n_n.data = (float *) &temp_matrix_data;
	temp_matrix_n_n.height = handler->number_of_states;
	temp_matrix_n_n.width = handler->number_of_states;

	// temp matrix2
	matrix_float temp_matrix2_n_n;
	float temp_matrix2_data[handler->number_of_states*handler->number_of_states];
	temp_matrix2_n_n.data = (float *) &temp_matrix2_data;
	temp_matrix2_n_n.height = handler->number_of_states;
	temp_matrix2_n_n.width = handler->number_of_states;

	// temp matrix3 for computing the kalman gain
	matrix_float temp_matrix3_u_n;
	float temp_matrix3_data[handler->number_of_inputs*handler->number_of_states];
	temp_matrix3_u_n.data = (float *) &temp_matrix3_data;
	temp_matrix3_u_n.height = handler->number_of_inputs;
	temp_matrix3_u_n.width = handler->number_of_states;

	// temp matrix4
	matrix_float temp_matrix4_u_u;
	float temp_matrix4_data[handler->number_of_inputs*handler->number_of_inputs];
	temp_matrix4_u_u.data = (float *) &temp_matrix4_data;
	temp_matrix4_u_u.height = handler->number_of_inputs;
	temp_matrix4_u_u.width = handler->number_of_inputs;

	/* -------------------------------------------------------------------- */
	/*	prediction step														*/
	/* -------------------------------------------------------------------- */

	// recompute new states

	// temp_states = a*handler->states;
	matrix_float_mul_vec_right(handler->system_A, handler_local.states, &temp_vector_n);

	vector_float_copy(handler_local.states, &temp_vector_n);

	// temp_states = temp_states + b*input
	matrix_float_mul_vec_right(handler->system_B, handler->input, &temp_vector_n);

	vector_float_add(handler_local.states, &temp_vector_n);

	// recompute covariance

	// temp_matrix = A*covariance
	matrix_float_mul(handler->system_A, handler_local.covariance, &temp_matrix_n_n);

	// temp_matrix2 = temp_matrix*A'
	matrix_float_mul_trans(&temp_matrix_n_n, handler->system_A, &temp_matrix2_n_n);

	// covariance = temp_matrix2 + R
	matrix_float_copy(handler_local.covariance, &temp_matrix2_n_n);
	matrix_float_add(handler_local.covariance, handler->R_matrix);

	/* -------------------------------------------------------------------- */
	/*	Correction step - Kalman Gain										*/
	/* -------------------------------------------------------------------- */

	// compute kalman gain
	// K = covariance*C'*((C*covariance*C' + Q)^-1)

	// temp_matrix3 = C*covariance
	matrix_float_mul(handler->C_matrix, handler_local.covariance, &temp_matrix3_u_n);

	// temp_matrix4 = temp_matrix3*C'
	matrix_float_mul_trans(&temp_matrix3_u_n, handler->C_matrix, &temp_matrix4_u_u);

	// temp_matrix4 += Q
	matrix_float_add(&temp_matrix4_u_u, handler->Q_matrix);

	// temp_matrix4^-1
	matrix_float_inverse(&temp_matrix4_u_u);

	// convert temp_matrix3 to n*u
	temp_matrix3_u_n.height = handler->number_of_states;
	temp_matrix3_u_n.width = handler->number_of_inputs;

	// temp_matrix3 = covariance*C'
	matrix_float_mul_trans(handler_local.covariance, handler->C_matrix, &temp_matrix3_u_n);

	// matrix for the kalman gain
	matrix_float K;
	float kalman_gain_data[handler->number_of_inputs*handler->number_of_states];
	K.data = (float *) &kalman_gain_data;
	K.height = handler->number_of_states;
	K.width = handler->number_of_inputs;

	// K = temp_matrix3*temp_matrix4
	matrix_float_mul(&temp_matrix3_u_n, &temp_matrix4_u_u, &K);

	// convert temp_matrix3 to original dimensions
	temp_matrix3_u_n.height = handler->number_of_inputs;
	temp_matrix3_u_n.width = handler->number_of_states;

	/* -------------------------------------------------------------------- */
	/*	Correction step - Recomputing states								*/
	/* -------------------------------------------------------------------- */

	// K = covariance*C'*((C*covariance*C' + Q)^-1);

	// temp_matrix2 = C*states
	matrix_float_mul_vec_right(handler->C_matrix, handler_local.states, &temp_vector2_u);

	// temp_vector2 = measurement - temp_vector2
	vector_float_times(&temp_vector2_u, (float) -1);

	vector_float_add(&temp_vector2_u, handler->measurement);

	// temp_vector = K*tem p_vector2
	matrix_float_mul_vec_right(&K, &temp_vector2_u, &temp_vector_n);

	// states = states + temp_vector2
	vector_float_add(handler_local.states, &temp_vector_n);

	/* -------------------------------------------------------------------- */
	/*	Correction step - Recomputing covariance						    */
	/* -------------------------------------------------------------------- */

	// covariance = (eye(n) - K*C)*covariance;

	// temp_matrix = K*C
	matrix_float_mul(&K, handler->C_matrix, &temp_matrix_n_n);

	// eye(n) - temp_matrix
	matrix_float_times(&temp_matrix_n_n, (float) -1);
	int i;
	for (i = 1; i <= temp_matrix_n_n.height; i++) {
		matrix_float_set(&temp_matrix_n_n, i, i, matrix_float_get(&temp_matrix_n_n, i, i) + (float) 1);
	}

	// temp_matrix2 = temp_matrix*covariance
	matrix_float_mul(&temp_matrix_n_n, handler_local.covariance, &temp_matrix2_n_n);

	/* -------------------------------------------------------------------- */
	/*	Copy output to the handler											*/
	/* -------------------------------------------------------------------- */

	portENTER_CRITICAL();

	vector_float_copy(handler->states, handler_local.states);
	matrix_float_copy(handler->covariance, &temp_matrix2_n_n);

	portEXIT_CRITICAL();
}
