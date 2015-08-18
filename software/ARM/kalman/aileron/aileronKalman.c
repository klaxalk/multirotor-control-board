/*
 * aileronKalman.c
 *
 *  Author: Tomas Baca
 */

#include "kalman/aileron/aileronKalman.h"
#include "miscellaneous.h"
#include "config.h"

kalmanHandler_t aileronKalmanHandler;

kalmanHandler_t * initializeAileronKalman() {

	/* -------------------------------------------------------------------- */
	/* System A matrix														*/
	/* -------------------------------------------------------------------- */
	aileronKalmanHandler.system_A = matrix_float_alloc(NUMBER_OF_STATES_AILERON, NUMBER_OF_STATES_AILERON);

	matrix_float_set(aileronKalmanHandler.system_A, 1, 1, 1);
	matrix_float_set(aileronKalmanHandler.system_A, 1, 2, DT_AILERON);
	matrix_float_set(aileronKalmanHandler.system_A, 1, 3, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 1, 4, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 1, 5, 0);

	matrix_float_set(aileronKalmanHandler.system_A, 2, 1, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 2, 2, 1);
	matrix_float_set(aileronKalmanHandler.system_A, 2, 3, DT_AILERON);
	matrix_float_set(aileronKalmanHandler.system_A, 2, 4, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 2, 5, 0);

	matrix_float_set(aileronKalmanHandler.system_A, 3, 1, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 3, 2, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 3, 3, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 3, 4, 1);
	matrix_float_set(aileronKalmanHandler.system_A, 3, 5, 1);

	matrix_float_set(aileronKalmanHandler.system_A, 4, 1, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 4, 2, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 4, 3, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 4, 4, ATTITUDE_P0);
	matrix_float_set(aileronKalmanHandler.system_A, 4, 5, 0);

	matrix_float_set(aileronKalmanHandler.system_A, 5, 1, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 5, 2, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 5, 3, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 5, 4, 0);
	matrix_float_set(aileronKalmanHandler.system_A, 5, 5, 1);

	/* -------------------------------------------------------------------- */
	/* System B matrix														*/
	/* -------------------------------------------------------------------- */
	aileronKalmanHandler.system_B = matrix_float_alloc(NUMBER_OF_STATES_AILERON, NUMBER_OF_INPUTS_AILERON);

	matrix_float_set(aileronKalmanHandler.system_B, 1, 1, 0);
	matrix_float_set(aileronKalmanHandler.system_B, 2, 1, 0);
	matrix_float_set(aileronKalmanHandler.system_B, 3, 1, 0);
	matrix_float_set(aileronKalmanHandler.system_B, 4, 1, ATTITUDE_P1);
	matrix_float_set(aileronKalmanHandler.system_B, 5, 1, 0);

	/* -------------------------------------------------------------------- */
	/*	Input vector														*/
	/* -------------------------------------------------------------------- */
	aileronKalmanHandler.input = vector_float_alloc(NUMBER_OF_INPUTS_AILERON, 0);

	/* -------------------------------------------------------------------- */
	/* Aileron kalman states vector											*/
	/* -------------------------------------------------------------------- */
	aileronKalmanHandler.states = vector_float_alloc(NUMBER_OF_STATES_AILERON, 0);
	vector_float_set_zero(aileronKalmanHandler.states);

	/* -------------------------------------------------------------------- */
	/* Aileron kalman covariance matrix										*/
	/* -------------------------------------------------------------------- */
	aileronKalmanHandler.covariance = matrix_float_alloc(NUMBER_OF_STATES_AILERON, NUMBER_OF_STATES_AILERON);
	matrix_float_set_identity(aileronKalmanHandler.covariance);

	/* -------------------------------------------------------------------- */
	/* Process noise matrix	(R)												*/
	/* -------------------------------------------------------------------- */
	aileronKalmanHandler.R_matrix = matrix_float_alloc(NUMBER_OF_STATES_AILERON, NUMBER_OF_STATES_AILERON);
	matrix_float_set_identity(aileronKalmanHandler.R_matrix);
	matrix_float_set(aileronKalmanHandler.R_matrix, 5, 5, 0.04);

	aileronKalmanHandler.number_of_inputs = NUMBER_OF_INPUTS_AILERON;
	aileronKalmanHandler.number_of_states = NUMBER_OF_STATES_AILERON;

	return &aileronKalmanHandler;
}
