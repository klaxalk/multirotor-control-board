/*
 * aileronMpc.c
 *
 *  Author: Tomas Baca
 */

#include "mpc/aileron/aileronMpc.h"
#include "mpc/elevator_and_aileron/elevAileMpcMatrices.h"
#include "mpc/mpc.h"
#include "miscellaneous.h"

mpcHandler_t aileronMpcHandler;

mpcHandler_t * initializeAileronMPC() {

	aileronMpcHandler.A_roof = matrix_float_alloc_hollow(ATTITUDE_A_ROOF_HEIGHT, ATTITUDE_A_ROOF_WIDTH, (float*) &A_roof_data_Attitude);

	aileronMpcHandler.Q_roof_diag = vector_float_alloc_hollow(ATTITUDE_Q_ROOF_DIAG_SIZE, 0, (float*) &Q_roof_diag_data_Attitude);

	aileronMpcHandler.B_roof = matrix_float_alloc_hollow(ATTITUDE_B_ROOF_HEIGHT, ATTITUDE_B_ROOF_WIDTH, (float*) &B_roof_data_Attitude);

	aileronMpcHandler.H_inv = matrix_float_alloc_hollow(ATTITUDE_H_INV_HEIGHT, ATTITUDE_H_INV_WIDTH, (float*) &H_inv_data_Attitude);

	aileronMpcHandler.position_reference = vector_float_alloc(ATTITUDE_HORIZON_LEN, 0);
	vector_float_set_zero(aileronMpcHandler.position_reference);

	aileronMpcHandler.allstate_reference = vector_float_alloc(ATTITUDE_ALLSTATE_REFERENCE_LEN, 0);
	vector_float_set_zero(aileronMpcHandler.allstate_reference);

	aileronMpcHandler.initial_cond = vector_float_alloc(ATTITUDE_NUMBER_OF_STATES, 0);

	aileronMpcHandler.dt = ATTITUDE_SYSTEM_DT;

	aileronMpcHandler.max_speed = ATTITUDE_MAX_SPEED;

	aileronMpcHandler.number_of_states = ATTITUDE_NUMBER_OF_STATES;

	aileronMpcHandler.horizon_len = ATTITUDE_HORIZON_LEN;

	aileronMpcHandler.reduced_horizon_len = ATTITUDE_REDUCED_HORIZON_LEN;

	return &aileronMpcHandler;
}
