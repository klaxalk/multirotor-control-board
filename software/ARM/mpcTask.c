/*
 * mpcTask.c
 *
 *  Author: Tomas Baca
 */

#include "mpcTask.h"
#include "kalmanTask.h"
#include "commTask.h"
#include "mpc/elevator/elevatorMpc.h"
#include "mpc/aileron/aileronMpc.h"

void mpcTask(void *p) {

	/* -------------------------------------------------------------------- */
	/*	Create handlers for all systems										*/
	/* -------------------------------------------------------------------- */
	mpcHandler_t * elevatorMpcHandler = initializeElevatorMPC();
	mpcHandler_t * aileronMpcHandler = initializeAileronMPC();

	/* -------------------------------------------------------------------- */
	/*	Messages between tasks												*/
	/* -------------------------------------------------------------------- */
	mpc2commMessage_t mpc2commMessage;
	kalman2mpcMessage_t kalman2mpcMessage;
	comm2mpcMessage_t comm2mpcMessage;

	vTaskDelay(100);

	while (1) {

		/* -------------------------------------------------------------------- */
		/*	If there is a message from commTask									*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(comm2mpcQueue, &comm2mpcMessage, 0)) {

			// copy the incoming set point/s into the local vector
			vector_float_set_to(elevatorMpcHandler->position_reference, comm2mpcMessage.elevatorReference);
			vector_float_set_to(aileronMpcHandler->position_reference, comm2mpcMessage.aileronReference);
		}

		/* -------------------------------------------------------------------- */
		/*	If there is a message from kalmanTask								*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(kalman2mpcQueue, &kalman2mpcMessage, 0)) {

			// copy the elevatorStates to states
			memcpy(elevatorMpcHandler->initial_cond->data, &kalman2mpcMessage.elevatorData, elevatorMpcHandler->number_of_states*sizeof(float));

			// filter the reference
			filterReferenceTrajectory(elevatorMpcHandler);

			// calculate the elevator MPC
			mpc2commMessage.elevatorOutput = calculateMPC(elevatorMpcHandler);

			// copy the elevatorStates to states
			memcpy(aileronMpcHandler->initial_cond->data, &kalman2mpcMessage.aileronData, aileronMpcHandler->number_of_states*sizeof(float));

			// filter the reference
			filterReferenceTrajectory(aileronMpcHandler);

			// calculate the aileron MPC
			mpc2commMessage.aileronOutput = calculateMPC(aileronMpcHandler);

			// send outputs to commTask
			xQueueOverwrite(mpc2commQueue, &mpc2commMessage);

			led_toggle();
		}
	}
}
