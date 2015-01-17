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

float dt = 0.015;

void controllersTask(void *p) {
	
	usartBufferPutString(usart_buffer_4, "---------------------------------\n\r", 10);

	while (1) {
		
		altitudeEstimator();
		
		if (altitudeControllerEnabled == true)
			altitudeController();
		
		// makes the 70Hz loop
		vTaskDelay((int16_t) dt*((float) 1000));
	}
}