/*
 * mpc.h
 *
 *  Author: Tomas Baca
 */

#ifndef MPC_H_
#define MPC_H_

#include "system.h"

typedef struct {

	matrix_float * A_roof;
	matrix_float * B_roof;
	vector_float * Q_roof_diag;
	matrix_float * H_inv;
	vector_float * initial_cond;
	vector_float * position_reference;
	vector_float * allstate_reference;
	int number_of_states;
	int horizon_len;
	int reduced_horizon_len;
	float dt;
	float max_speed;

} mpcHandler_t;

void filterReferenceTrajectory(mpcHandler_t * handler);

float calculateMPC(mpcHandler_t * handler);

#endif /* MPC_H_ */
