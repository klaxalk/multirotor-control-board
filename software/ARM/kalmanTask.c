/*
 * kalmanTask.c
 *
 *  Author: Tomas Baca
 */

#include "system.h"
#include "kalmanTask.h"
#include "kalman/kalman.h"
#include "kalman/elevator/elevatorKalman.h"
#include "kalman/aileron/aileronKalman.h"
#include "config.h"

void kalmanTask(void *p) {

	kalmanHandler_t * aileronKalmanHandler = initializeAileronKalman();
	kalmanHandler_t * elevatorKalmanHandler = initializeElevatorKalman();

	/* -------------------------------------------------------------------- */
	/* Vector for 1-state measurement										*/
	/* -------------------------------------------------------------------- */
	vector_float * measurement_1_state = vector_float_alloc(1, 0);

	/* -------------------------------------------------------------------- */
	/* px4flow speed measurement noise matrix	(Q)	(1-state)				*/
	/* -------------------------------------------------------------------- */
	matrix_float * px4flow_Q_matrix_1_state = matrix_float_alloc(1, 1);

	matrix_float_set(px4flow_Q_matrix_1_state, 1, 1, KALMAN_Q);

	/* -------------------------------------------------------------------- */
	/* px4flow speed C matrix (transfer measurements -> states)				*/
	/* -------------------------------------------------------------------- */
	matrix_float * px4flow_C_matrix_1_state = matrix_float_alloc(1, NUMBER_OF_STATES_ELEVATOR);

	matrix_float_set(px4flow_C_matrix_1_state, 1, 1, 0);
	matrix_float_set(px4flow_C_matrix_1_state, 1, 2, 1);
	matrix_float_set(px4flow_C_matrix_1_state, 1, 3, 0);
	matrix_float_set(px4flow_C_matrix_1_state, 1, 4, 0);
	matrix_float_set(px4flow_C_matrix_1_state, 1, 5, 0);

	/* -------------------------------------------------------------------- */
	/* Messages between tasks												*/
	/* -------------------------------------------------------------------- */
	comm2kalmanMessage_t comm2kalmanMessage;
	kalman2mpcMessage_t kalman2mpcMessage;
	kalman2commMessage_t kalman2commMesasge;
	resetKalmanMessage_t resetKalmanMessage;

	while (1) {

		if (xQueueReceive(resetKalmanQueue, &resetKalmanMessage, 0)) {

			// reset state vectors
			vector_float_set_zero(elevatorKalmanHandler->states);
			vector_float_set_zero(aileronKalmanHandler->states);

			// set the default position
			vector_float_set(elevatorKalmanHandler->states, 1, resetKalmanMessage.elevatorPosition);
			vector_float_set(aileronKalmanHandler->states, 1, resetKalmanMessage.aileronPosition);

			// reset the covariance matrices
			matrix_float_set_identity(elevatorKalmanHandler->covariance);
			matrix_float_set_identity(aileronKalmanHandler->covariance);
		}

		if (xQueueReceive(setKalmanQueue, &resetKalmanMessage, 0)) {

			// set the position
			vector_float_set(elevatorKalmanHandler->states, 1, resetKalmanMessage.elevatorPosition);
			vector_float_set(aileronKalmanHandler->states, 1, resetKalmanMessage.aileronPosition);

			// reset the covariance of the postion
			matrix_float_set(elevatorKalmanHandler->covariance, 1, 1, 1);
			matrix_float_set(aileronKalmanHandler->covariance, 1, 1, 1);
		}

		if (xQueueReceive(comm2kalmanQueue, &comm2kalmanMessage, 0)) {

			/* -------------------------------------------------------------------- */
			/*	Compute elevator kalman												*/
			/* -------------------------------------------------------------------- */

			// set the input vector
			vector_float_set(elevatorKalmanHandler->input, 1, comm2kalmanMessage.elevatorInput);

			// set the measurement vector
			vector_float_set(measurement_1_state, 1, comm2kalmanMessage.elevatorSpeed);

			// set pointers to measurement related matrices
			elevatorKalmanHandler->measurement = measurement_1_state;
			elevatorKalmanHandler->C_matrix = px4flow_C_matrix_1_state;
			elevatorKalmanHandler->Q_matrix = px4flow_Q_matrix_1_state;

			kalmanIteration(elevatorKalmanHandler);

			/* -------------------------------------------------------------------- */
			/*	Compute aileron kalman												*/
			/* -------------------------------------------------------------------- */

			// set the input vector
			vector_float_set(aileronKalmanHandler->input, 1, comm2kalmanMessage.aileronInput);

			// set the measurement vector
			vector_float_set(measurement_1_state, 1, comm2kalmanMessage.aileronSpeed);

			// set pointers to measurement related matrices
			aileronKalmanHandler->measurement = measurement_1_state;
			aileronKalmanHandler->C_matrix = px4flow_C_matrix_1_state;
			aileronKalmanHandler->Q_matrix = px4flow_Q_matrix_1_state;

			kalmanIteration(aileronKalmanHandler);

			/* -------------------------------------------------------------------- */
			/*	Create a message for mpcTask										*/
			/* -------------------------------------------------------------------- */

			memcpy(&kalman2mpcMessage.elevatorData, elevatorKalmanHandler->states->data, NUMBER_OF_STATES_ELEVATOR*sizeof(float));
			memcpy(&kalman2mpcMessage.aileronData, aileronKalmanHandler->states->data, NUMBER_OF_STATES_AILERON*sizeof(float));

			xQueueOverwrite(kalman2mpcQueue, &kalman2mpcMessage);

			/* -------------------------------------------------------------------- */
			/*	Create a message for commTask										*/
			/* -------------------------------------------------------------------- */

			memcpy(&kalman2commMesasge.elevatorData, elevatorKalmanHandler->states->data, NUMBER_OF_STATES_ELEVATOR*sizeof(float));
			memcpy(&kalman2commMesasge.aileronData, aileronKalmanHandler->states->data, NUMBER_OF_STATES_AILERON*sizeof(float));

			xQueueOverwrite(kalman2commQueue, &kalman2commMesasge);
		}
	}
}
