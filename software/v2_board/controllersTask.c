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

void controllersTask(void *p) {
	
	float data_A[6*6] = {1, 1, 2, 3, 4, 5,
						 1, 2, 3, 3, 1, 2,
						 1, 1, 4, 4, 3, 2,
						 3, 4, 1, 2, 4, 5,
						 1, 2, 3, 1, 2, 3,
						 3, 3, 3, 2, 1, 3};
	matrix_float a_matrix;
	a_matrix.data = (float *) data_A;
	a_matrix.height = 6;
	a_matrix.width = 6;
	a_matrix.name = "A";
	
	// matrix_float_print(&a_matrix, usart_buffer_4);
	
	matrix_float_inverse(&a_matrix);
	
	matrix_float_print(&a_matrix, usart_buffer_4);
	
	while (1) {
		
		altitudeEstimator();
		
		if (altitudeControllerEnabled == true)
			altitudeController();
		
		// makes the 70Hz loop
		vTaskDelay(14);
	}
}