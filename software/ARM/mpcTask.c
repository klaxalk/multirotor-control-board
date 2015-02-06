/*
 * mpcTask.c
 *
 *  Author: Tomas Baca
 */

#include "mpcTask.h"
#include "elevAileMpcMatrices.h"
#include "uart_driver.h"
#include "system.h"
#include "miscellaneous.h"
#include "kalmanTask.h"
#include "commTask.h"
#include "mpc.h"

// precomputed matrices
matrix_float A_roof;
vector_float Q_roof_diag;
matrix_float B_roof;
matrix_float H_inv;

vector_float reference;
vector_float elevator_true_reference;
vector_float aileron_true_reference;

vector_float states;

void filterAttitudeReference(vector_float * in) {

	int i;
	float diference;

	for (i = 2; i <= in->length; i++) {

		diference = vector_float_get(&reference, (i-2)*NUMBER_OF_STATES + 1) - vector_float_get(in, i);

		if (diference > MAX_SPEED*DT)
			diference = MAX_SPEED*DT;
		else if (diference < -MAX_SPEED*DT)
			diference = -MAX_SPEED*DT;

		vector_float_set(&reference, (i-1)*NUMBER_OF_STATES + 1, vector_float_get(&reference, (i-2)*NUMBER_OF_STATES + 1) - diference);
	}
}

void mpcTask(void *p) {

	// setup the A_roof matrix
	A_roof.data = (float*) &A_roof_data;
	A_roof.height =	A_ROOF_HEIGHT;
	A_roof.width = A_ROOF_WIDTH;
	A_roof.name = "A_roof matrix";

	// setup the Q_roof_diag vector
	Q_roof_diag.data = (float*) &Q_roof_diag_data;
	Q_roof_diag.length = Q_ROOF_DIAG_SIZE;
	Q_roof_diag.name = "Q_roof_diag vector";
	Q_roof_diag.orientation = 0;

	// setup the B_roof matrix
	B_roof.data = (float*) &B_roof_data;
	B_roof.height =	B_ROOF_HEIGHT;
	B_roof.width = B_ROOF_WIDTH;
	B_roof.name = "B_roof matrix";

	// setup the H_inv matrix
	H_inv.data = (float*) &H_inv_data;
	H_inv.height =	H_INV_HEIGHT;
	H_inv.width = H_INV_WIDTH;
	H_inv.name = "H_inv matrix";

	// setup the reference vector (with all states)
	float reference_data[REFERENCE_LENGTH];
	reference.data = (float*) reference_data;
	reference.length = REFERENCE_LENGTH;
	reference.name = "allstates reference vector";
	reference.orientation = 0;
	vector_float_set_zero(&reference);

	// the elevator axis true reference (unfiltered)
	float elevator_true_reference_data[HORIZON_LEN];
	elevator_true_reference.data = (float*) elevator_true_reference_data;
	elevator_true_reference.length = HORIZON_LEN;
	elevator_true_reference.name = "true elevator reference";
	elevator_true_reference.orientation = 0;
	vector_float_set_zero(&elevator_true_reference);

	// the aileron axis true reference (unfiltered)
	float aileron_true_reference_data[HORIZON_LEN];
	aileron_true_reference.data = (float*) aileron_true_reference_data;
	aileron_true_reference.length = HORIZON_LEN;
	aileron_true_reference.name = "true aileron reference";
	aileron_true_reference.orientation = 0;
	vector_float_set_zero(&aileron_true_reference);

	// current state vector
	float states_data[NUMBER_OF_STATES];
	states.data = (float*) &states_data;
	states.length = NUMBER_OF_STATES;
	states.name = "Current states";
	states.orientation = 0;

	// message with outputs to send to commTask
	mpc2commMessage_t mpc2commMessage;

	// message received from kalmanTask
	kalman2mpcMessage_t kalman2mpcMessage;

	// mesasge received from commTask
	comm2mpcMessage_t comm2mpcMessage;

	vTaskDelay(100);

	while (1) {

		/* -------------------------------------------------------------------- */
		/*	If there is a message from commTask									*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(comm2mpcQueue, &comm2mpcMessage, 0)) {

			// copy the incoming set point/s into the local vector
			vector_float_set_to(&elevator_true_reference, comm2mpcMessage.elevatorReference);
			vector_float_set_to(&aileron_true_reference, comm2mpcMessage.aileronReference);
		}

		/* -------------------------------------------------------------------- */
		/*	If there is a message from kalmanTask								*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(kalman2mpcQueue, &kalman2mpcMessage, 0)) {

			// copy the elevatorStates to states
			memcpy(states.data, &kalman2mpcMessage.elevatorData, NUMBER_OF_STATES*sizeof(float));

			// filter the reference
			vector_float_set(&reference, 1, vector_float_get(&states, 1));
			filterAttitudeReference(&elevator_true_reference);

			// calculate the elevator MPC
			mpc2commMessage.elevatorOutput = calculateMPC(&A_roof, &B_roof, &Q_roof_diag, &H_inv, &states, &reference, NUMBER_OF_STATES, HORIZON_LEN, REDUCED_HORIZON);

			// copy the elevatorStates to states
			memcpy(states.data, &kalman2mpcMessage.aileronData, NUMBER_OF_STATES*sizeof(float));

			// filter the reference
			vector_float_set(&reference, 1, vector_float_get(&states, 1));
			filterAttitudeReference(&aileron_true_reference);

			// calculate the aileron MPC
			mpc2commMessage.aileronOutput = calculateMPC(&A_roof, &B_roof, &Q_roof_diag, &H_inv, &states, &reference, NUMBER_OF_STATES, HORIZON_LEN, REDUCED_HORIZON);

			// send outputs to commTask
			xQueueOverwrite(mpc2commQueue, &mpc2commMessage);

			led_toggle();
		}
	}
}
