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

#include "multiCon.h"
#include "mpcHandler.h"
#include "battery.h"

#define DELAY_MILISECONDS 33

void logTask(void *p) {
	
	char temp[20];
	
	vTaskDelay(2000);
	
	while (1) {
		
		sprintf(temp, "%2.3f, ", kalmanStates.elevator.position); // 1
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.aileron.position); // 2
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
		
		sprintf(temp, "%2.3f, ", estimatedThrottlePos); //13
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", RCchannel[THROTTLE]); //14
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", controllerThrottleOutput); //15
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", altitudeControllerEnabled); //16
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%d, ", mpcControllerEnabled); //17
		usartBufferPutString(usart_buffer_log, temp, 10);

		sprintf(temp, "%d, ", opticalFlowData.quality);//18
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", estimatedThrottleVel);//19
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", estimatedThrottlePos_prev);//20
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		#ifdef MULTICON
		sprintf(temp, "%d, ", numberOfDetectedBlobs); //21
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", blobs[0].x);//22
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", blobs[0].y);//23
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", blobs[0].z);//24
		usartBufferPutString(usart_buffer_log, temp, 10);
		#endif
		
		sprintf(temp, "%2.3f, ", getBatteryVoltage()); //25
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.position); //26
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.velocity); //27
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		sprintf(temp, "%2.3f, ", kalmanStates.throttle.acceleration); //28
		usartBufferPutString(usart_buffer_log, temp, 10);
		
		usartBufferPutByte(usart_buffer_log, '\n', 10);//29
		
		// makes the 50Hz loop
		vTaskDelay(DELAY_MILISECONDS);
	}
}