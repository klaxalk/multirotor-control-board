/*
 * logTask.c
 *
 * Created: 28.1.2015 0:44:35
 *  Author: klaxalk
 */

#include "logTask.h"
#include "system.h"
#include "controllers.h"
#include "communication.h"
#include <stdio.h>

#define DELAY_MILISECONDS 14

void logTask(void *p) {
	
	char temp[20];
	
	vTaskDelay(2000);
	
	while (1) {
		
		sprintf(temp, "%2.3f, ", kalmanStates.elevator.position);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.aileron.position);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.5f, ", kalmanStates.elevator.velocity);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.5f, ", kalmanStates.aileron.velocity);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.5f, ", elevatorSpeed);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.5f, ", aileronSpeed);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerElevatorOutput);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerAileronOutput);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", groundDistance);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerThrottleOutput);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d", mpcControllerEnabled);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		usartBufferPutString(usart_buffer_log, "\n", 10);
				
		// makes the 70Hz loop
		vTaskDelay(DELAY_MILISECONDS);
	}
}