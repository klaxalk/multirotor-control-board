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

	elevatorMpcHandler.A_roof = matrix_float_alloc_hollow(A_ROOF_HEIGHT, A_ROOF_WIDTH, (float*) &A_roof_data_Attitude);

	elevatorMpcHandler.Q_roof_diag = vector_float_alloc_hollow(Q_ROOF_DIAG_SIZE, 0, (float*) &Q_roof_diag_data_Attitude);

	elevatorMpcHandler.B_roof = matrix_float_alloc_hollow(B_ROOF_HEIGHT, B_ROOF_WIDTH, (float*) &B_roof_data_Attitude);

	elevatorMpcHandler.H_inv = matrix_float_alloc_hollow(H_INV_HEIGHT, H_INV_WIDTH, (float*) &H_inv_data_Attitude);

	elevatorMpcHandler.position_reference = vector_float_alloc(HORIZON_LEN, 0);

	elevatorMpcHandler.allstate_reference = vector_float_alloc(ALLSTATE_REFERENCE_LEN, 0);

	elevatorMpcHandler.initial_cond = vector_float_alloc(NUMBER_OF_STATES, 0);

	elevatorMpcHandler.dt = ATTITUDE_SYSTEM_DT;

	elevatorMpcHandler.max_speed = ATTITUDE_MAX_SPEED;

	return &elevatorMpcHandler;
}
