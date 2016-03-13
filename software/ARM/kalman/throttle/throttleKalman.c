/*
 * config.h
 *
 *  Author: Frantisek Puciow
 */

#include "kalman/throttle/throttleKalman.h"
#include "miscellaneous.h"
#include "config.h"

kalmanHandler_t throttleKalmanHandler;

kalmanHandler_t * initializeThrottleKalman() {

	/* -------------------------------------------------------------------- */
	/* System A matrix														*/
	/* -------------------------------------------------------------------- */

	/*    --                --
	      | 1  DT   0   0  0 |
	      | 0   1  DT   0  0 |
	  A = | 0   0   0   1  1 |
	      | 0   0 A43 A44  0 |
	      | 0   0   0   0  1 |
	      --                -- */

	throttleKalmanHandler.system_A = matrix_float_alloc(NUMBER_OF_STATES_THROTTLE, NUMBER_OF_STATES_THROTTLE);

	matrix_float_set(throttleKalmanHandler.system_A, 1, 1, 1);
	matrix_float_set(throttleKalmanHandler.system_A, 1, 2, DT_THROTTLE);
	matrix_float_set(throttleKalmanHandler.system_A, 1, 3, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 1, 4, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 1, 5, 0);

	matrix_float_set(throttleKalmanHandler.system_A, 2, 1, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 2, 2, 1);
	matrix_float_set(throttleKalmanHandler.system_A, 2, 3, DT_THROTTLE);
	matrix_float_set(throttleKalmanHandler.system_A, 2, 4, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 2, 5, 0);

	matrix_float_set(throttleKalmanHandler.system_A, 3, 1, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 3, 2, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 3, 3, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 3, 4, 1);
	matrix_float_set(throttleKalmanHandler.system_A, 3, 5, 1);

	matrix_float_set(throttleKalmanHandler.system_A, 4, 1, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 4, 2, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 4, 3, THROTTLE_A43);
	matrix_float_set(throttleKalmanHandler.system_A, 4, 4, THROTTLE_A44);
	matrix_float_set(throttleKalmanHandler.system_A, 4, 5, 0);

	matrix_float_set(throttleKalmanHandler.system_A, 5, 1, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 5, 2, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 5, 3, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 5, 4, 0);
	matrix_float_set(throttleKalmanHandler.system_A, 5, 5, 1);

	/* -------------------------------------------------------------------- */
	/* System B matrix														*/
	/* -------------------------------------------------------------------- */

	/*    --  --
	      |  0 |
          |  0 |
	  B = | B3 |
	      | B4 |
	      |  0 |
	      --  -- */

	throttleKalmanHandler.system_B = matrix_float_alloc(NUMBER_OF_STATES_THROTTLE, NUMBER_OF_INPUTS_THROTTLE);

	matrix_float_set(throttleKalmanHandler.system_B, 1, 1, 0);

	matrix_float_set(throttleKalmanHandler.system_B, 2, 1, 0);

	matrix_float_set(throttleKalmanHandler.system_B, 3, 1, THROTTLE_B3);

	matrix_float_set(throttleKalmanHandler.system_B, 4, 1, THROTTLE_B4);

	matrix_float_set(throttleKalmanHandler.system_B, 5, 1, 0);

	/* -------------------------------------------------------------------- */
	/*	Input vector														*/
	/* -------------------------------------------------------------------- */

	/*    --     --
      u = | INPUT |
	      --     -- */

	throttleKalmanHandler.input = vector_float_alloc(NUMBER_OF_INPUTS_THROTTLE, 0);

	/* -------------------------------------------------------------------- */
	/* Throttle kalman states vector										*/
	/* -------------------------------------------------------------------- */

	/*    --   --
	      |   h |
          |  dh |
	  x = | ddh |
	      |   w |
	      |   e |
	      --   -- */

	throttleKalmanHandler.states = vector_float_alloc(NUMBER_OF_STATES_THROTTLE, 0);
	vector_float_set_zero(throttleKalmanHandler.states);

	/* -------------------------------------------------------------------- */
	/* Throttle kalman covariance matrix									*/
	/* -------------------------------------------------------------------- */

	/*    --         --
	      | 1 0 0 0 0 |
          | 0 1 0 0 0 |
	  E = | 0 0 1 0 0 |
	      | 0 0 0 1 0 |
	      | 0 0 0 0 1 |
	      --         -- */

	throttleKalmanHandler.covariance = matrix_float_alloc(NUMBER_OF_STATES_THROTTLE, NUMBER_OF_STATES_THROTTLE);
	matrix_float_set_identity(throttleKalmanHandler.covariance);

	/* -------------------------------------------------------------------- */
	/* Process C matrix (transfer measurements -> states)					*/
	/* -------------------------------------------------------------------- */

	/*    --         --
      C = | 1 0 0 0 0 |
	      --         -- */

	throttleKalmanHandler.C_matrix = matrix_float_alloc(1, NUMBER_OF_STATES_THROTTLE);

	matrix_float_set(throttleKalmanHandler.C_matrix, 1, 1, 1);
	matrix_float_set(throttleKalmanHandler.C_matrix, 1, 2, 0);
	matrix_float_set(throttleKalmanHandler.C_matrix, 1, 3, 0);
	matrix_float_set(throttleKalmanHandler.C_matrix, 1, 4, 0);
	matrix_float_set(throttleKalmanHandler.C_matrix, 1, 5, 0);

	/* -------------------------------------------------------------------- */
	/* Process noise matrix	(R)												*/
	/* -------------------------------------------------------------------- */

	/*    --         --
	      | 1 0 0 0 0 |
          | 0 1 0 0 0 |
	  R = | 0 0 1 0 0 |
	      | 0 0 0 1 0 |
	      | 0 0 0 0 1 |
	      --         -- */

	throttleKalmanHandler.R_matrix = matrix_float_alloc(NUMBER_OF_STATES_THROTTLE, NUMBER_OF_STATES_THROTTLE);
	matrix_float_set_identity(throttleKalmanHandler.R_matrix);

	/* -------------------------------------------------------------------- */
	/* Measurement noise matrix	(Q)											*/
	/* -------------------------------------------------------------------- */

	/*    --          --
      Q = | THROTTLE_Q |
	      --          -- */

	throttleKalmanHandler.Q_matrix = matrix_float_alloc(1, 1);

	matrix_float_set(throttleKalmanHandler.C_matrix, 1, 1, THROTTLE_Q);

	throttleKalmanHandler.number_of_measurements = NUMBER_OF_MEASUREMENTS_THROTTLE;
	throttleKalmanHandler.number_of_inputs = NUMBER_OF_INPUTS_THROTTLE;
	throttleKalmanHandler.number_of_states = NUMBER_OF_STATES_THROTTLE;

	return &throttleKalmanHandler;
}
