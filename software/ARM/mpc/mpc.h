/*
 * mpc.h
 *
 *  Author: Tomas Baca
 */

#ifndef MPC_H_
#define MPC_H_

#include "system.h"

float calculateMPC(matrix_float * A_roof, matrix_float * B_roof, vector_float * Q_roof_diag, matrix_float * H_inv, vector_float * states, vector_float * reference, int16_t number_of_states, int16_t horizon_len, int16_t reduced_horizon);

#endif /* MPC_H_ */
