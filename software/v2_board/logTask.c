#include <stdio.h>
#include "logTask.h"
#include "packets.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"
#include "commTask.h"
#include "mpcHandler.h"

char str[10];
char *strBin;

void logValueStr(float lValue){
	sprintf(str,"%.2f,",lValue);    
	usartBufferPutString(usart_buffer_log,str,10);		
}

void logTaskReadable(void *p){
	vTaskDelay(100);	
	
	while(1){	
		logValueStr(altitude.position); //altitude								3
		logValueStr(altitude.speed);												//4
		logValueStr(kalmanStates.elevator.position+positionShift.elevator); //elevator 5
		logValueStr(kalmanStates.elevator.velocity);								//6
		logValueStr(kalmanStates.elevator.acceleration);							//7
		logValueStr(positionShift.elevator);											//8
		logValueStr(kalmanStates.aileron.position+positionShift.aileron); //aileron		9
		logValueStr(kalmanStates.aileron.velocity);										//10
		logValueStr(kalmanStates.aileron.acceleration);									//11
		logValueStr(positionShift.aileron);											//12
		logValueStr(validGumstix); //blob detector									//13
		logValueStr(elevatorGumstix);												//14
		logValueStr(aileronGumstix);
		logValueStr(throttleGumstix); 
		logValueStr((float)controllerActive); //controller		
		logValueStr(setpoints.altitude);  //setpoints
		logValueStr(setpoints.elevator);
		logValueStr(setpoints.aileron);
		logValueStr((float)outputThrottle); //outputs
		logValueStr((float)outputElevator);
		logValueStr((float)outputAileron);
		logValueStr((float)outputRudder);
		logValueStr((float)RCchannel[THROTTLE]); //RC IN
		logValueStr((float)RCchannel[ELEVATOR]);
		logValueStr((float)RCchannel[AILERON]);
		logValueStr((float)RCchannel[RUDDER]);
		logValueStr((float)milisecondsTimer); //time	
		sprintf(str,"%d,",(int)secondsTimer); 
		usartBufferPutString(usart_buffer_log,str,10); 
		logValueStr(mpcSetpoints.elevator);
		logValueStr(mpcSetpoints.aileron);
		logValueStr(kalmanStates.elevator.acceleration_error);
		logValueStr(kalmanStates.aileron.acceleration_error);		  
		
				
		usartBufferPutString(usart_buffer_log,"\r\n",10);
		vTaskDelay(40);	
	}
}
