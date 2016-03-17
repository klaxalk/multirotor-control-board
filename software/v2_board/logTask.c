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

#define DELAY_MILISECONDS 20

void logTask(void *p) { // only logs throttle-relevant data
	
	char temp[20];
	
	int16_t startTimeMillis, endTimeMillis, delayMillis;
	
	vTaskDelay(2000);
	
	while (1) {
		startTimeMillis = milisecondsTimer;
		
		sprintf(temp, "%d, ", (flightStarted*2) + kalmanStarted); //1
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", groundDistance); //2
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", getBatteryVoltage()); //3
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", outputChannels[0]); //4
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", groundDistanceConfidence); //5
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.position); //6
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.velocity); //7
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.acceleration); //8
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.omega); //9
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.acceleration_error); //10
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanFit); //11
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", conoutput); //12
		usartBufferPutString(usart_buffer_log, temp, 10);
				
		
		usartBufferPutByte(usart_buffer_log, '\n', 10);//13
		
		endTimeMillis = milisecondsTimer;
		
		if (startTimeMillis > endTimeMillis) //timer overflowed
		delayMillis = 1000 + endTimeMillis - startTimeMillis;
		else
		delayMillis = endTimeMillis - startTimeMillis;
		
		vTaskDelay((int16_t) ((int16_t) DELAY_MILISECONDS - delayMillis));// makes the 50Hz loop
	}
}