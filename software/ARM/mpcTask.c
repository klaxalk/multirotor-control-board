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
		while (xQueueReceive(comm2mpcQueue, &comm2mpcMessage, 0)) {

			if (comm2mpcMessage.messageType == SETPOINT) {

				// copy the incoming set point/s into the local vector
				vector_float_set_to(elevatorMpcHandler->position_reference, comm2mpcMessage.elevatorReference[0]);
				vector_float_set_to(aileronMpcHandler->position_reference, comm2mpcMessage.aileronReference[0]);

			} else if (comm2mpcMessage.messageType == TRAJECTORY) {

				int i, j;
				float step;

				// for all key-point in the elevator reference
				for (j = 0; j < 4; j++) {

					// calculate the interpolation step
					step = (comm2mpcMessage.elevatorReference[j+1] - comm2mpcMessage.elevatorReference[j])/50;

					vector_float_set(elevatorMpcHandler->position_reference, j*50+1, comm2mpcMessage.elevatorReference[j]);

					for (i = 1; i < 50; i++) {

						vector_float_set(elevatorMpcHandler->position_reference, j*50+1+i, vector_float_get(elevatorMpcHandler->position_reference, j*50+i) + step);
					}
				}

				// for all key-point in the aileron reference
				for (j = 0; j < 4; j++) {

					// calculate the interpolation step
					step = (comm2mpcMessage.aileronReference[j+1] - comm2mpcMessage.aileronReference[j])/50;

					vector_float_set(aileronMpcHandler->position_reference, j*50+1, comm2mpcMessage.aileronReference[j]);

					for (i = 1; i < 50; i++) {

						vector_float_set(aileronMpcHandler->position_reference, j*50+1+i, vector_float_get(aileronMpcHandler->position_reference, j*50+i) + step);
					}
				}
			}
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

			// copy the current setpoint (main for debug)
			mpc2commMessage.elevatorSetpoint = vector_float_get(elevatorMpcHandler->position_reference, 1);
			mpc2commMessage.aileronSetpoint = vector_float_get(aileronMpcHandler->position_reference, 1);

			// send outputs to commTask
			xQueueOverwrite(mpc2commQueue, &mpc2commMessage);

			led_toggle();
		}
	}
}
