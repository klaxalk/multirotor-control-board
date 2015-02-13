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

#include "mpcHandler.h"

#define DELAY_MILISECONDS 33

void logTask(void *p) {
	
	char temp[20];
	
	vTaskDelay(2000);
	
	while (1) {
		
		sprintf(temp, "%2.3f, ", kalmanStates.elevator.position);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.aileron.position);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.elevator.velocity);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.aileron.velocity);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.elevator.acceleration_error);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.aileron.acceleration_error);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", mpcSetpoints.elevatorSetpoint);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", mpcSetpoints.aileronSetpoint);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.aileron.position);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", elevatorSpeed);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", aileronSpeed);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerElevatorOutput);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerAileronOutput);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", opticalFlowData.ground_distance);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", RCchannel[THROTTLE]);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerThrottleOutput);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", altitudeControllerEnabled);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d", mpcControllerEnabled);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		usartBufferPutString(usart_buffer_log, "\n", 10);
				
		// makes the 50Hz loop
		vTaskDelay(DELAY_MILISECONDS);
	}
}