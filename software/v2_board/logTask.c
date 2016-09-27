/*
* logTask.c
*
* Created: 28.1.2015 0:44:35
*  Author: klaxalk
*/

#include "logTask.h"
#include "system.h"
#include "controllers.h"
#include "controllersTask.h"
#include "communication.h"
#include <stdio.h>

#include "multiCon.h"
#include "mpcHandler.h"
#include "battery.h"

#define DELAY_MILISECONDS 50

void logTask(void *p) { // only logs throttle-relevant data
	
	char temp[20];
	
	int16_t startTimeMillis, endTimeMillis, delayMillis;
	
	vTaskDelay(2000);
	
	while (1) {
		startTimeMillis = milisecondsTimer;
		
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
		
		sprintf(temp, "%2.3f, ", mpcSetpoints.elevator);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", mpcSetpoints.aileron);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", elevatorSpeed);
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", aileronSpeed); //10
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerElevatorOutput); //11
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerAileronOutput); //12
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", (flightStarted*2) + kalmanStarted); //13
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", groundDistance); //14
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", getBatteryVoltage()); //15
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", outputChannels[0]); //16
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", groundDistanceConfidence); //17
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.position); //18
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.velocity); //19
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.acceleration); //20
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.omega); //21
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.acceleration_error); //22
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanFit); //23
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerThrottleOutput); //24
		usartBufferPutString(usart_buffer_log, temp, 10);
				
		sprintf(temp, "%2.3f, ", opticalFlowData.ground_distance); //25
		usartBufferPutString(usart_buffer_log, temp, 10);				
		
		usartBufferPutByte(usart_buffer_log, '\n', 10);//end
		
		endTimeMillis = milisecondsTimer;
		
		if (startTimeMillis > endTimeMillis) //timer overflowed
		delayMillis = 1000 + endTimeMillis - startTimeMillis;
		else
		delayMillis = endTimeMillis - startTimeMillis;
		
		vTaskDelay((int16_t) ((int16_t) DELAY_MILISECONDS - delayMillis));// makes the 20Hz loop
	}
}