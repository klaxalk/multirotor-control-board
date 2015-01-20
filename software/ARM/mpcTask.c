/*
 * mpcTask.c
 *
 *  Author: Tomas Baca
 */

#include "mpcTask.h"
#include "mpcMatrices.h"
#include "uart_driver.h"
#include "system.h"
#include "miscellaneous.h"
#include "kalmanTask.h"

// precomputed matrices
matrix_float A_roof;
vector_float Q_roof_diag;
matrix_float B_roof;
matrix_float H_inv;

// aux matrices
vector_float temp_vector1;
vector_float temp_vector2;
vector_float temp_vector3;

vector_float states;

void calculateMPC() {

	int i;

	// tenp_vector1 <- A_roof*states
	matrix_float_mul_vec_right(&A_roof, &states, &temp_vector1);

	//
	// now subtract the state_reference vector
	//

	// X_0'*Q_roof
	for (i = 1; i <= NUMBER_OF_STATES*HORIZON_LEN; i++) {

		vector_float_set(&temp_vector1, i, vector_float_get(&temp_vector1, i) * vector_float_get(&Q_roof_diag, i));
	}

	vector_float_transpose(&temp_vector1);

	// c = (X_0'*Q_roof)*B_roof
	matrix_float_mul_vec_left(&B_roof, &temp_vector1, &temp_vector2);

	vector_float_transpose(&temp_vector2);

	// c./(-2)
	vector_float_times(&temp_vector2, (float) -0.5);

	// H_inv*(c./(-2))
	matrix_float_mul_vec_right(&H_inv, &temp_vector2, &temp_vector3);
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

	// temp_vector1
	float temp_vector1_data[NUMBER_OF_STATES*HORIZON_LEN];
	temp_vector1.data = (float*) &temp_vector1_data;
	temp_vector1.length = NUMBER_OF_STATES*HORIZON_LEN;
	temp_vector1.orientation = 0;
	temp_vector1.name = "temp_vector1";

	// temp_vector2
	float temp_vector2_data[REDUCED_HORIZON];
	temp_vector2.data = (float*) &temp_vector2_data;
	temp_vector2.length = REDUCED_HORIZON;
	temp_vector2.orientation = 1;
	temp_vector2.name = "temp_vector2";

	// temp_vector3
	float temp_vector3_data[REDUCED_HORIZON];
	temp_vector3.data = (float*) &temp_vector3_data;
	temp_vector3.length = REDUCED_HORIZON;
	temp_vector3.orientation = 0;
	temp_vector3.name = "temp_vector3";

	// current state vector
	float states_data[NUMBER_OF_STATES];
	states.data = (float*) &states_data;
	states.length = NUMBER_OF_STATES;
	states.name = "Current states";
	states.orientation = 0;

	// message with outputs to send to comms
	mpcOutputMessage newMessage;

	vTaskDelay(100);

	kalman2mpcMessage kalmanMessage;

	while (1) {

		if (xQueueReceive(kalman2mpcQueue, &kalmanMessage, 100)) {

			// copy the elevatorStates to states
			memccpy(states.data, &kalmanMessage.elevatorData, NUMBER_OF_STATES, sizeof(float));

			calculateMPC();

			newMessage.elevatorOutput = vector_float_get(&temp_vector3, 1);

			// copy the elevatorStates to states
			memccpy(states.data, &kalmanMessage.aileronData, NUMBER_OF_STATES, sizeof(float));

			calculateMPC();

			newMessage.aileronOutput = vector_float_get(&temp_vector3, 1);

			// send outputs to comms
			xQueueSend(mpc2commQueue, &newMessage, 10);
		}
	}
}
