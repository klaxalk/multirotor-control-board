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

void kalmanTask(void *p) {

	initializeAileronKalman();
	initializeElevatorKalman();

	float dt;

	/* -------------------------------------------------------------------- */
	/* px4flow 1D measurement vector										*/
	/* -------------------------------------------------------------------- */
	vector_float * px4flow_measurement = vector_float_alloc(1, 0);

	/* -------------------------------------------------------------------- */
	/* px4flow Measurement noise matrix	(Q)									*/
	/* -------------------------------------------------------------------- */
	matrix_float * px4flow_Q_matrix = matrix_float_alloc(1, 1);
	matrix_float_set(px4flow_Q_matrix, 1, 1, 100);

	/* -------------------------------------------------------------------- */
	/* px4flow C matrix (transfere measurements -> states)					*/
	/* -------------------------------------------------------------------- */
	matrix_float * px4flow_C_matrix = matrix_float_alloc(1, NUMBER_OF_STATES_ELEVATOR);

	matrix_float_set(px4flow_C_matrix, 1, 1, 0);
	matrix_float_set(px4flow_C_matrix, 1, 2, 1);
	matrix_float_set(px4flow_C_matrix, 1, 3, 0);

	/* -------------------------------------------------------------------- */
	/* Messages between tasks												*/
	/* -------------------------------------------------------------------- */
	comm2kalmanMessage_t comm2kalmanMessage;
	kalman2mpcMessage_t kalman2mpcMessage;
	kalman2commMessage_t kalman2commMesasge;

	while (1) {

		if (xQueueReceive(comm2kalmanQueue, &comm2kalmanMessage, 0)) {

			dt = comm2kalmanMessage.dt;

			/* -------------------------------------------------------------------- */
			/*	Compute elevator kalman												*/
			/* -------------------------------------------------------------------- */

			// set the input vector
			vector_float_set(elevatorKalmanHandler.input, 1, comm2kalmanMessage.elevatorInput);

			// set the measurement vector
			vector_float_set(px4flow_measurement, 1, comm2kalmanMessage.elevatorSpeed);

			// set pointers to measurement related matrices
			elevatorKalmanHandler.measurement = px4flow_measurement;
			elevatorKalmanHandler.C_matrix = px4flow_C_matrix;
			elevatorKalmanHandler.Q_matrix = px4flow_Q_matrix;

			kalmanIteration(&elevatorKalmanHandler);

			/* -------------------------------------------------------------------- */
			/*	Compute aileron kalman												*/
			/* -------------------------------------------------------------------- */

			// set the input vector
			vector_float_set(aileronKalmanHandler.input, 1, comm2kalmanMessage.aileronInput);

			// set the measurement vector
			vector_float_set(px4flow_measurement, 1, comm2kalmanMessage.aileronSpeed);

			// set pointers to measurement related matrices
			aileronKalmanHandler.measurement = px4flow_measurement;
			aileronKalmanHandler.C_matrix = px4flow_C_matrix;
			aileronKalmanHandler.Q_matrix = px4flow_Q_matrix;

			kalmanIteration(&aileronKalmanHandler);

			/* -------------------------------------------------------------------- */
			/*	Create a message for mpcTask										*/
			/* -------------------------------------------------------------------- */

			memcpy(&kalman2mpcMessage.elevatorData, elevatorKalmanHandler.states->data, NUMBER_OF_STATES_ELEVATOR*sizeof(float));
			memcpy(&kalman2mpcMessage.aileronData, aileronKalmanHandler.states->data, NUMBER_OF_STATES_AILERON*sizeof(float));

			xQueueOverwrite(kalman2mpcQueue, &kalman2mpcMessage);

			/* -------------------------------------------------------------------- */
			/*	Create a message for commTask										*/
			/* -------------------------------------------------------------------- */

			memcpy(&kalman2commMesasge.elevatorData, elevatorKalmanHandler.states->data, NUMBER_OF_STATES_ELEVATOR*sizeof(float));
			memcpy(&kalman2commMesasge.aileronData, aileronKalmanHandler.states->data, NUMBER_OF_STATES_AILERON*sizeof(float));

			xQueueOverwrite(kalman2commQueue, &kalman2commMesasge);
		}
	}
}
