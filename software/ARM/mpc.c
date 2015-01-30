/*
 * mpc.c
 *
 *  Author: Tomas Baca
 */

#include "mpc.h"

float calculateMPC(matrix_float * A_roof, matrix_float * B_roof, vector_float * Q_roof_diag, matrix_float * H_inv, vector_float * states, vector_float * reference, int16_t number_of_states, int16_t horizon_len, int16_t reduced_horizon) {

	int i;

	/* -------------------------------------------------------------------- */
	/*	Allocate temp vectors for partresults								*/
	/* -------------------------------------------------------------------- */

	vector_float temp_vector1;
	float temp_vector1_data[number_of_states*horizon_len];
	temp_vector1.data = (float*) &temp_vector1_data;
	temp_vector1.length = number_of_states*horizon_len;
	temp_vector1.orientation = 0;

	vector_float temp_vector2;
	float temp_vector2_data[reduced_horizon];
	temp_vector2.data = (float*) &temp_vector2_data;
	temp_vector2.length = reduced_horizon;
	temp_vector2.orientation = 1;

	vector_float temp_vector3;
	float temp_vector3_data[reduced_horizon];
	temp_vector3.data = (float*) &temp_vector3_data;
	temp_vector3.length = reduced_horizon;
	temp_vector3.orientation = 0;

	/* -------------------------------------------------------------------- */
	/*	Procede the MPC														*/
	/* -------------------------------------------------------------------- */

	// temp_vector1 <- A_roof*states
	matrix_float_mul_vec_right(A_roof, states, &temp_vector1);

	//temp_vector1 <- temp_vector1 - reference
	vector_float_subtract(&temp_vector1, reference);

	// X_0'*Q_roof
	// simplified product of a vector and a diagonal matrix Q_roof
	for (i = 1; i <= number_of_states*horizon_len; i++)
		vector_float_set(&temp_vector1, i, vector_float_get(&temp_vector1, i) * vector_float_get(Q_roof_diag, i));

	vector_float_transpose(&temp_vector1);

	// c = (X_0'*Q_roof)*B_roof
	matrix_float_mul_vec_left(B_roof, &temp_vector1, &temp_vector2);

	vector_float_transpose(&temp_vector2);

	// c./(-2)
	vector_float_times(&temp_vector2, (float) -0.5);

	// H_inv*(c./(-2))
	matrix_float_mul_vec_right(H_inv, &temp_vector2, &temp_vector3);

	// return the first value of the action vector
	return vector_float_get(&temp_vector3, 1);
}
