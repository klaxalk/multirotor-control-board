/*
 * mpc.c
 *
 *  Author: Tomas Baca
 */

#include "mpc.h"

void filterReferenceTrajectory(mpcHandler_t * handler) {

	float difference;

	vector_float_set(handler->allstate_reference, 1, vector_float_get(handler->initial_cond, 1));

	int i;
	for (i = 2; i <= handler->position_reference->length; i++) {

		// compute the difference
		difference = vector_float_get(handler->allstate_reference, (i-2)*handler->number_of_states + 1) - vector_float_get(handler->position_reference, i);

		// saturate the difference
		if (difference > handler->max_speed*handler->dt)
			difference = handler->max_speed*handler->dt;
		else if (difference < -handler->max_speed*handler->dt)
			difference = -handler->max_speed*handler->dt;

		vector_float_set(handler->allstate_reference, (i-1)*handler->number_of_states + 1, vector_float_get(handler->allstate_reference, (i-2)*handler->number_of_states + 1) - difference);
	}
}

float calculateMPC(mpcHandler_t * handler) {

	/* -------------------------------------------------------------------- */
	/*	Allocate temp vectors for partresults								*/
	/* -------------------------------------------------------------------- */

	vector_float temp_vector1;
	float temp_vector1_data[handler->number_of_states*handler->horizon_len];
	temp_vector1.data = (float*) &temp_vector1_data;
	temp_vector1.length = handler->number_of_states*handler->horizon_len;
	temp_vector1.orientation = 0;

	vector_float temp_vector2;
	float temp_vector2_data[handler->reduced_horizon_len];
	temp_vector2.data = (float*) &temp_vector2_data;
	temp_vector2.length = handler->reduced_horizon_len;
	temp_vector2.orientation = 1;

	vector_float temp_vector3;
	float temp_vector3_data[handler->reduced_horizon_len];
	temp_vector3.data = (float*) &temp_vector3_data;
	temp_vector3.length = handler->reduced_horizon_len;
	temp_vector3.orientation = 0;

	/* -------------------------------------------------------------------- */
	/*	Procede the MPC														*/
	/* -------------------------------------------------------------------- */

	// temp_vector1 <- A_roof*states
	matrix_float_mul_vec_right(handler->A_roof, handler->initial_cond, &temp_vector1);

	//temp_vector1 <- temp_vector1 - reference
	vector_float_subtract(&temp_vector1, handler->allstate_reference);

	// X_0'*Q_roof
	// simplified product of a vector and a diagonal matrix Q_roof
	int i;
	for (i = 1; i <= handler->number_of_states*handler->horizon_len; i++)
		vector_float_set(&temp_vector1, i, vector_float_get(&temp_vector1, i) * vector_float_get(handler->Q_roof_diag, i));

	vector_float_transpose(&temp_vector1);

	// c = (X_0'*Q_roof)*B_roof
	matrix_float_mul_vec_left(handler->B_roof, &temp_vector1, &temp_vector2);

	vector_float_transpose(&temp_vector2);

	// c./(-2)
	vector_float_times(&temp_vector2, (float) -0.5);

	// H_inv*(c./(-2))
	matrix_float_mul_vec_right(handler->H_inv, &temp_vector2, &temp_vector3);

	// return the first value of the action vector
	return vector_float_get(&temp_vector3, 1);
}
