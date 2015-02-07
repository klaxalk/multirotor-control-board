/*
 * elevatorKalman.c
 *
 *  Author: Tomas Baca
 */

#include "kalman/elevator/elevatorKalman.h"
#include "miscellaneous.h"

kalmanHandler_t elevatorKalmanHandler;

kalmanHandler_t * initializeElevatorKalman() {

	/* -------------------------------------------------------------------- */
	/* System A matrix														*/
	/* -------------------------------------------------------------------- */
	elevatorKalmanHandler.system_A = matrix_float_alloc(NUMBER_OF_STATES_ELEVATOR, NUMBER_OF_STATES_ELEVATOR);

	matrix_float_set(elevatorKalmanHandler.system_A, 1, 1, 1);
	matrix_float_set(elevatorKalmanHandler.system_A, 1, 2, DT_ELEVATOR);
	matrix_float_set(elevatorKalmanHandler.system_A, 1, 3, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 1, 4, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 1, 5, 0);

	matrix_float_set(elevatorKalmanHandler.system_A, 2, 1, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 2, 2, 1);
	matrix_float_set(elevatorKalmanHandler.system_A, 2, 3, DT_ELEVATOR);
	matrix_float_set(elevatorKalmanHandler.system_A, 2, 4, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 2, 5, 0);

	matrix_float_set(elevatorKalmanHandler.system_A, 3, 1, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 3, 2, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 3, 3, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 3, 4, 1);
	matrix_float_set(elevatorKalmanHandler.system_A, 3, 5, 1);

	matrix_float_set(elevatorKalmanHandler.system_A, 4, 1, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 4, 2, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 4, 3, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 4, 4, DT_ELEVATOR*82.2945);
	matrix_float_set(elevatorKalmanHandler.system_A, 4, 5, 0);

	matrix_float_set(elevatorKalmanHandler.system_A, 5, 1, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 5, 2, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 5, 3, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 5, 4, 0);
	matrix_float_set(elevatorKalmanHandler.system_A, 5, 5, 1);

	/* -------------------------------------------------------------------- */
	/* System B matrix														*/
	/* -------------------------------------------------------------------- */
	elevatorKalmanHandler.system_B = matrix_float_alloc(NUMBER_OF_STATES_ELEVATOR, NUMBER_OF_INPUTS_ELEVATOR);

	matrix_float_set(elevatorKalmanHandler.system_B, 1, 1, 0);
	matrix_float_set(elevatorKalmanHandler.system_B, 2, 1, 0);
	matrix_float_set(elevatorKalmanHandler.system_B, 3, 1, 0);
	matrix_float_set(elevatorKalmanHandler.system_B, 4, 1, 0.0113*DT_ELEVATOR);
	matrix_float_set(elevatorKalmanHandler.system_B, 5, 1, 0);

	/* -------------------------------------------------------------------- */
	/*	Input vector														*/
	/* -------------------------------------------------------------------- */
	elevatorKalmanHandler.input = vector_float_alloc(NUMBER_OF_INPUTS_ELEVATOR, 0);

	/* -------------------------------------------------------------------- */
	/* elevator kalman states vector										*/
	/* -------------------------------------------------------------------- */
	elevatorKalmanHandler.states = vector_float_alloc(NUMBER_OF_STATES_ELEVATOR, 0);
	vector_float_set_zero(elevatorKalmanHandler.states);

	/* -------------------------------------------------------------------- */
	/* elevator kalman covariance matrix									*/
	/* -------------------------------------------------------------------- */
	elevatorKalmanHandler.covariance = matrix_float_alloc(NUMBER_OF_STATES_ELEVATOR, NUMBER_OF_STATES_ELEVATOR);
	matrix_float_set_identity(elevatorKalmanHandler.covariance);

	/* -------------------------------------------------------------------- */
	/* Process noise matrix	(R)												*/
	/* -------------------------------------------------------------------- */
	elevatorKalmanHandler.R_matrix = matrix_float_alloc(NUMBER_OF_STATES_ELEVATOR, NUMBER_OF_STATES_ELEVATOR);
	matrix_float_set_identity(elevatorKalmanHandler.R_matrix);
	matrix_float_set(elevatorKalmanHandler.R_matrix, 5, 5, 0.001);

	elevatorKalmanHandler.number_of_inputs = NUMBER_OF_INPUTS_ELEVATOR;
	elevatorKalmanHandler.number_of_states = NUMBER_OF_STATES_ELEVATOR;

	return &elevatorKalmanHandler;
}
