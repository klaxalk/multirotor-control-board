/*
 * controllersTask.c
 *
 * Created: 13.9.2014 23:57:14
 *  Author: Tomas Baca
 */ 

#include "controllersTask.h"
#include "controllers.h"
#include "system.h"
#include "config.h"
#include "kalman.h"
#include "matrixLib.h"

float dt = 0.015;

void controllersTask(void *p) {
	
	usartBufferPutString(usart_buffer_4, "---------------------------------\n\r", 10);
	
	/* -------------------------------------------------------------------- */
	/* System A matrix														*/
	/* -------------------------------------------------------------------- */
	matrix_float A_matrix;
	A_matrix.name = "System matrix A";
	A_matrix.height = 3;
	A_matrix.width = 3;
	float data_A[3*3] = {1, dt, 0,
						 0, 1, 2.4239*dt,
						 0, 0, 1};
	A_matrix.data = (float *) data_A;
	
	/* -------------------------------------------------------------------- */
	/* System A_transposed matrix											*/
	/* -------------------------------------------------------------------- */
	matrix_float A_tran_matrix;
	A_tran_matrix.name = "System matrix A";
	A_tran_matrix.height = 3;
	A_tran_matrix.width = 3;
	float data_A_tran[3*3];
	A_tran_matrix.data = (float *) data_A_tran;
	matrix_float_copy(&A_tran_matrix, &A_matrix);
	matrix_float_transpose_square(&A_tran_matrix);
	
	/* --------------------------------------------------------------	------ */
	/* System B vector														*/
	/* -------------------------------------------------------------------- */
	vector_float B_vector;
	B_vector.name = "System vector B";
	B_vector.length = 3;
	B_vector.orientation = 0;
	float data_B[3] = {0, 0, 0.1219*dt};
	B_vector.data = (float *) data_B;
	
	/* -------------------------------------------------------------------- */
	/* Kalman states vector													*/
	/* -------------------------------------------------------------------- */
	vector_float states_vector;
	states_vector.name = "Kalman states vector";
	states_vector.length = 3;
	states_vector.orientation = 0;
	float data_states[3];
	states_vector.data = (float *) data_states;
	
	/* -------------------------------------------------------------------- */
	/* Kalman covariance matrix												*/
	/* -------------------------------------------------------------------- */
	matrix_float covariance_matrix;
	covariance_matrix.name = "Kalman covariance matrix";
	covariance_matrix.height = 3;
	covariance_matrix.width = 3;
	float data_covariance[3*3];
	covariance_matrix.data = (float *) data_covariance;
	
	/* -------------------------------------------------------------------- */
	/* Measurement vector													*/
	/* -------------------------------------------------------------------- */
	vector_float measurement_vector;
	measurement_vector.name = "Measurements vector";
	measurement_vector.length = 1;
	measurement_vector.orientation = 0;
	float data_measurement[1];
	measurement_vector.data = (float *) data_measurement;
	
	/* -------------------------------------------------------------------- */
	/* Process noise matrix	(R)												*/
	/* -------------------------------------------------------------------- */
	matrix_float R_matrix;
	R_matrix.name = "R matrix";
	R_matrix.width = 3;
	R_matrix.height = 3;
	float R_data[3*3] = {1, 0, 0,
						 0, 1, 0,
						 0, 0, 5};
	R_matrix.data = (float *) R_data;
	
	/* -------------------------------------------------------------------- */
	/* Measurement noise matrix	(Q)											*/
	/* -------------------------------------------------------------------- */
	matrix_float Q_matrix;
	Q_matrix.name = "Q matrix";
	Q_matrix.height = 1;
	Q_matrix.width = 1;
	float Q_data[1] = {0.058*0.058};
	Q_matrix.data = (float *) Q_data;
	
	/* -------------------------------------------------------------------- */
	/* C matrix (transfare inputs -> states)								*/
	/* -------------------------------------------------------------------- */
	matrix_float C_matrix;
	C_matrix.name = "C matrix";
	C_matrix.height = 1;
	C_matrix.width = 3;
	float data_C[3*1] = {1, 0, 0};
	C_matrix.data = (float *) data_C;
	
	/* -------------------------------------------------------------------- */
	/* C transposed matrix													*/
	/* -------------------------------------------------------------------- */
	matrix_float C_tran_matrix;
	C_tran_matrix.name = "C matrix";
	C_tran_matrix.height = C_matrix.width;
	C_tran_matrix.width = C_matrix.height;
	float data_C_tran[C_matrix.width*C_matrix.height];
	C_tran_matrix.data = (float *) data_C_tran;
	matrix_float_transpose(&C_matrix, &C_tran_matrix);
	
	/* -------------------------------------------------------------------- */
	/* Input vector													*/
	/* -------------------------------------------------------------------- */
	vector_float input_vector;
	input_vector.name = "Inputs vector";
	input_vector.length = 1;
	input_vector.orientation = 0;
	float data_input[1];
	input_vector.data = (float *) data_input;
	
	kalmanHandler elevatorHandler;
	elevatorHandler.covariance = &covariance_matrix;
	elevatorHandler.states = &states_vector;
	
	kalmanInit(elevatorHandler);
	
	/*
	matrix_float_print(&A_matrix, usart_buffer_4);
	matrix_float_print(&A_tran_matrix, usart_buffer_4);
	vector_float_print(&B_vector, usart_buffer_4);
	vector_float_print(&states_vector, usart_buffer_4);
	vector_float_print(elevatorHandler.states, usart_buffer_4);
	matrix_float_print(elevatorHandler.covariance, usart_buffer_4);
	matrix_float_print(&R_matrix, usart_buffer_4);
	vector_float_print(&Q_vector, usart_buffer_4);
	matrix_float_print(&C_matrix, usart_buffer_4);
	matrix_float_print(&C_tran_	matrix, usart_buffer_4);
	*/
	
	//int i;
	//for (i = 0; i < 10; i++) {
		
		kalmanIteration(elevatorHandler, &measurement_vector, &input_vector, &A_matrix, &A_tran_matrix, &B_vector, &R_matrix, &Q_matrix, &C_matrix, &C_tran_matrix, dt);
		
		// vector_float_print(elevatorHandler.states, usart_buffer_4);
	//}
	
	while (1) {
		
		altitudeEstimator();
		
		if (altitudeControllerEnabled == true)
			altitudeController();
		
		// makes the 70Hz loop
		vTaskDelay((int16_t) dt*((float) 1000));
	}
}