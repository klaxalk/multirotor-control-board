/*
 * kalmanTask.c
 *
 *  Author: Tomas Baca
 */

#include "kalmanTask.h"
#include "system.h"
#include "kalman.h"
#include "CMatrixLib.h"
#include "uart_driver.h"

float dt = 0.0114;
#define NUMBER_OF_STATES 3
#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_MEASURED_STATES 1

kalmanHandler elevatorHandler;
kalmanHandler aileronHandler;

void kalmanTask(void *p) {

	/* -------------------------------------------------------------------- */
	/* System A matrix														*/
	/* -------------------------------------------------------------------- */
	matrix_float A_matrix;
	A_matrix.name = "System matrix A";
	A_matrix.height = NUMBER_OF_STATES;
	A_matrix.width = NUMBER_OF_STATES;
	float data_A[NUMBER_OF_STATES*NUMBER_OF_STATES] = {1, dt, 0,
													   0, 1, dt,
													   0, 0, dt*82.6135};
	A_matrix.data = (float *) data_A;

	/* -------------------------------------------------------------------- */
	/* System B matrix														*/
	/* -------------------------------------------------------------------- */
	matrix_float B_matrix;
	B_matrix.name = "System matrix B";
	B_matrix.height = NUMBER_OF_STATES;
	B_matrix.width = NUMBER_OF_INPUTS;
	float data_B[NUMBER_OF_STATES*NUMBER_OF_INPUTS] = {0, 0, 0.0105*dt};
	B_matrix.data = (float *) data_B;

	/* -------------------------------------------------------------------- */
	/* Elevator kalman states vector										*/
	/* -------------------------------------------------------------------- */
	vector_float elevator_states_vector;
	elevatorHandler.states = &elevator_states_vector;
	elevator_states_vector.name = "Elevator kalman states vector";
	elevator_states_vector.length = NUMBER_OF_STATES;
	elevator_states_vector.orientation = 0;
	float data_elevator_states[NUMBER_OF_STATES];
	elevator_states_vector.data = (float *) data_elevator_states;

	/* -------------------------------------------------------------------- */
	/* Aileron kalman states vector											*/
	/* -------------------------------------------------------------------- */
	vector_float aileron_states_vector;
	aileronHandler.states = &aileron_states_vector;
	aileron_states_vector.name = "Aileron kalman states vector";
	aileron_states_vector.length = NUMBER_OF_STATES;
	aileron_states_vector.orientation = 0;
	float data_aileron_states[NUMBER_OF_STATES];
	aileron_states_vector.data = (float *) data_aileron_states;

	/* -------------------------------------------------------------------- */
	/* Elevator kalman covariance matrix												*/
	/* -------------------------------------------------------------------- */
	matrix_float elevator_covariance_matrix;
	elevatorHandler.covariance = &elevator_covariance_matrix;
	elevator_covariance_matrix.name = "Elevator kalman covariance matrix";
	elevator_covariance_matrix.height = NUMBER_OF_STATES;
	elevator_covariance_matrix.width = NUMBER_OF_STATES;
	float data_elevator_covariance[NUMBER_OF_STATES*NUMBER_OF_STATES];
	elevator_covariance_matrix.data = (float *) data_elevator_covariance;

	/* -------------------------------------------------------------------- */
	/* Aileron kalman covariance matrix												*/
	/* -------------------------------------------------------------------- */
	matrix_float aileron_covariance_matrix;
	aileronHandler.covariance = &aileron_covariance_matrix;
	aileron_covariance_matrix.name = "Aileron kalman covariance matrix";
	aileron_covariance_matrix.height = NUMBER_OF_STATES;
	aileron_covariance_matrix.width = NUMBER_OF_STATES;
	float data_aileron_covariance[NUMBER_OF_STATES*NUMBER_OF_STATES];
	aileron_covariance_matrix.data = (float *) data_aileron_covariance;

	/* -------------------------------------------------------------------- */
	/* Measurement vector													*/
	/* -------------------------------------------------------------------- */
	vector_float measurement_vector;
	measurement_vector.name = "Measurements vector";
	measurement_vector.length = NUMBER_OF_MEASURED_STATES;
	measurement_vector.orientation = 0;
	float data_measurement[NUMBER_OF_MEASURED_STATES];
	measurement_vector.data = (float *) data_measurement;

	/* -------------------------------------------------------------------- */
	/* Process noise matrix	(R)												*/
	/* -------------------------------------------------------------------- */
	matrix_float R_matrix;
	R_matrix.name = "R matrix";
	R_matrix.width = NUMBER_OF_STATES;
	R_matrix.height = NUMBER_OF_STATES;
	float R_data[NUMBER_OF_STATES*NUMBER_OF_STATES] = {1, 0, 0,
													   0, 1, 0,
													   0, 0, 5};
	R_matrix.data = (float *) R_data;

	/* -------------------------------------------------------------------- */
	/* Measurement noise matrix	(Q)											*/
	/* -------------------------------------------------------------------- */
	matrix_float Q_matrix;
	Q_matrix.name = "Q matrix";
	Q_matrix.height = NUMBER_OF_MEASURED_STATES;
	Q_matrix.width = NUMBER_OF_MEASURED_STATES;
	float Q_data[NUMBER_OF_MEASURED_STATES*NUMBER_OF_MEASURED_STATES] = {1000};
	Q_matrix.data = (float *) Q_data;

	/* -------------------------------------------------------------------- */
	/* C matrix (transfare measurements -> states)							*/
	/* -------------------------------------------------------------------- */
	matrix_float C_matrix;
	C_matrix.name = "C matrix";
	C_matrix.height = NUMBER_OF_MEASURED_STATES;
	C_matrix.width = NUMBER_OF_STATES;
	float data_C[NUMBER_OF_MEASURED_STATES*NUMBER_OF_STATES] = {0, 1, 0};
	C_matrix.data = (float *) data_C;

	/* -------------------------------------------------------------------- */
	/* Aux matrices															*/
	/* -------------------------------------------------------------------- */

	// temp vector
	vector_float temp_vector_n;
	float temp_vector_data[NUMBER_OF_STATES];
	temp_vector_n.name = "temp_vector";
	temp_vector_n.data = (float *) &temp_vector_data;
	temp_vector_n.length = NUMBER_OF_STATES;
	temp_vector_n.orientation = 0;

	// temp vector2
	vector_float temp_vector2_u;
	float temp_vector2_data[NUMBER_OF_INPUTS];
	temp_vector2_u.data = (float *) &temp_vector2_data;
	temp_vector2_u.length = NUMBER_OF_INPUTS;
	temp_vector2_u.orientation = 0;

	// temp matrix
	matrix_float temp_matrix_n_n;
	float temp_matrix_data[NUMBER_OF_STATES*NUMBER_OF_STATES];
	temp_matrix_n_n.name = "temp_matrix";
	temp_matrix_n_n.data = (float *) &temp_matrix_data;
	temp_matrix_n_n.height = NUMBER_OF_STATES;
	temp_matrix_n_n.width = NUMBER_OF_STATES;

	// temp matrix2
	matrix_float temp_matrix2_n_n;
	float temp_matrix2_data[NUMBER_OF_STATES*NUMBER_OF_STATES];
	temp_matrix2_n_n.name = "temp_matrix2";
	temp_matrix2_n_n.data = (float *) &temp_matrix2_data;
	temp_matrix2_n_n.height = NUMBER_OF_STATES;
	temp_matrix2_n_n.width = NUMBER_OF_STATES;

	// temp matrix for computing the kalman gain
	matrix_float temp_matrix3_u_n;
	float temp_matrix3_data[NUMBER_OF_INPUTS*NUMBER_OF_STATES];
	temp_matrix3_u_n.name = "temp_matrix3";
	temp_matrix3_u_n.data = (float *) &temp_matrix3_data;
	temp_matrix3_u_n.height = NUMBER_OF_INPUTS;
	temp_matrix3_u_n.width = NUMBER_OF_STATES;

	matrix_float temp_matrix4_u_u;
	float temp_matrix4_data[NUMBER_OF_INPUTS*NUMBER_OF_INPUTS];
	temp_matrix4_u_u.name = "temp_matrix4";
	temp_matrix4_u_u.data = (float *) &temp_matrix4_data;
	temp_matrix4_u_u.height = NUMBER_OF_INPUTS;
	temp_matrix4_u_u.width = NUMBER_OF_INPUTS;

	elevatorHandler.number_of_states = NUMBER_OF_STATES;
	elevatorHandler.number_of_inputs = NUMBER_OF_INPUTS;

	aileronHandler.number_of_states = NUMBER_OF_STATES;
	aileronHandler.number_of_inputs = NUMBER_OF_INPUTS;

	/* -------------------------------------------------------------------- */
	/*	Input vector														*/
	/* -------------------------------------------------------------------- */
	vector_float input_vector;
	input_vector.name = "Inputs vector";
	input_vector.length = 1;
	input_vector.orientation = 0;
	float data_input[1];
	input_vector.data = (float *) data_input;

	/* -------------------------------------------------------------------- */
	/*	Copy temp matrices													*/
	/* -------------------------------------------------------------------- */

	elevatorHandler.temp_matrix_n_n = &temp_matrix_n_n;
	elevatorHandler.temp_matrix2_n_n = &temp_matrix2_n_n;
	elevatorHandler.temp_matrix3_u_n = &temp_matrix3_u_n;
	elevatorHandler.temp_matrix4_u_u = &temp_matrix4_u_u;
	elevatorHandler.temp_vector_n = &temp_vector_n;
	elevatorHandler.temp_vector_u = &temp_vector2_u;

	aileronHandler.temp_matrix_n_n = &temp_matrix_n_n;
	aileronHandler.temp_matrix2_n_n = &temp_matrix2_n_n;
	aileronHandler.temp_matrix3_u_n = &temp_matrix3_u_n;
	aileronHandler.temp_matrix4_u_u = &temp_matrix4_u_u;
	aileronHandler.temp_vector_n = &temp_vector_n;
	aileronHandler.temp_vector_u = &temp_vector2_u;

	kalmanInit(&elevatorHandler);
	kalmanInit(&aileronHandler);

	px4flowMessage message;

	kalman2mpcMessage mpcMessage;

	while (1) {

		if (xQueueReceive(comm2kalmanQueue, &message, 100)) {

			dt = message.dt;

			/* -------------------------------------------------------------------- */
			/*	Prepare matrices													*/
			/* -------------------------------------------------------------------- */

			// update the A_matrix
			matrix_float_set(&A_matrix, 1, 2, dt);
			matrix_float_set(&A_matrix, 2, 3, dt);
			matrix_float_set(&A_matrix, 3, 3, dt*82.6135);

			// update the B_matrix
			matrix_float_set(&B_matrix, 3, 1, 0.0105*dt);

			/* -------------------------------------------------------------------- */
			/*	Compute elevator kalman												*/
			/* -------------------------------------------------------------------- */

			// set the input vector
			vector_float_set(&input_vector, 1, message.elevatorInput);

			// set the measurement vector
			vector_float_set(&measurement_vector, 1, message.elevatorSpeed);

			//portENTER_CRITICAL();
			kalmanIteration(&elevatorHandler, &measurement_vector, &input_vector, &A_matrix, &B_matrix, &R_matrix, &Q_matrix, &C_matrix, dt);
			//portEXIT_CRITICAL();

			/* -------------------------------------------------------------------- */
			/*	Compute aileron kalman												*/
			/* -------------------------------------------------------------------- */

			// set the input vector
			vector_float_set(&input_vector, 1, message.aileronInput);

			// set the measurement vector
			vector_float_set(&measurement_vector, 1, message.aileronSpeed);

			//portENTER_CRITICAL();
			kalmanIteration(&aileronHandler, &measurement_vector, &input_vector, &A_matrix, &B_matrix, &R_matrix, &Q_matrix, &C_matrix, dt);
			//portEXIT_CRITICAL();

			/* -------------------------------------------------------------------- */
			/*	Create a message for mpc task										*/
			/* -------------------------------------------------------------------- */

			//portENTER_CRITICAL();
			memccpy(&mpcMessage.elevatorData, elevatorHandler.states->data, NUMBER_OF_STATES, sizeof(float));
			memccpy(&mpcMessage.aileronData, aileronHandler.states->data, NUMBER_OF_STATES, sizeof(float));
			//portEXIT_CRITICAL();

			xQueueOverwrite(kalman2mpcQueue, &mpcMessage);
		}
	}
}
