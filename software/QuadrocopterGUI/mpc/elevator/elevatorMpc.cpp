/*
 * elevatorMpc.c
 *
 *  Author: Tomas Baca
 */

#include "mpc/elevator/elevatorMpc.h"
#include "mpc/elevator_and_aileron/elevAileMpcMatrices.h"
#include "mpc/mpc.h"
#include "miscellaneous.h"

mpcHandler_t elevatorMpcHandler;

mpcHandler_t * initializeElevatorMPC() {

	elevatorMpcHandler.A_roof = matrix_float_alloc_hollow(ATTITUDE_A_ROOF_HEIGHT, ATTITUDE_A_ROOF_WIDTH, (float*) &A_roof_data_Attitude);

	elevatorMpcHandler.Q_roof_diag = vector_float_alloc_hollow(ATTITUDE_Q_ROOF_DIAG_SIZE, 0, (float*) &Q_roof_diag_data_Attitude);

	elevatorMpcHandler.B_roof = matrix_float_alloc_hollow(ATTITUDE_B_ROOF_HEIGHT, ATTITUDE_B_ROOF_WIDTH, (float*) &B_roof_data_Attitude);

	elevatorMpcHandler.H_inv = matrix_float_alloc_hollow(ATTITUDE_H_INV_HEIGHT, ATTITUDE_H_INV_WIDTH, (float*) &H_inv_data_Attitude);

	elevatorMpcHandler.position_reference = vector_float_alloc(ATTITUDE_HORIZON_LEN, 0);
	vector_float_set_zero(elevatorMpcHandler.position_reference);

	elevatorMpcHandler.allstate_reference = vector_float_alloc(ATTITUDE_ALLSTATE_REFERENCE_LEN, 0);
	vector_float_set_zero(elevatorMpcHandler.allstate_reference);

	elevatorMpcHandler.initial_cond = vector_float_alloc(ATTITUDE_NUMBER_OF_STATES, 0);

	elevatorMpcHandler.dt = ATTITUDE_SYSTEM_DT;

	elevatorMpcHandler.max_speed = ATTITUDE_MAX_SPEED;

	elevatorMpcHandler.number_of_states = ATTITUDE_NUMBER_OF_STATES;

	elevatorMpcHandler.horizon_len = ATTITUDE_HORIZON_LEN;

	elevatorMpcHandler.reduced_horizon_len = ATTITUDE_REDUCED_HORIZON_LEN;

	return &elevatorMpcHandler;
}
