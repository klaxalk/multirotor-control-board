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
		logValueStr(altitude.position); //altitude									//1
		logValueStr(altitude.speed);												//2
		logValueStr(kalmanStates.elevator.position+positionShift.elevator); //elevator 3
		logValueStr(kalmanStates.elevator.velocity);								//4
		logValueStr(kalmanStates.elevator.acceleration);							//5
		logValueStr(positionShift.elevator);										//6
		logValueStr(kalmanStates.aileron.position+positionShift.aileron); //aileron	//7
		logValueStr(kalmanStates.aileron.velocity);									//8
		logValueStr(kalmanStates.aileron.acceleration);								//9
		logValueStr(positionShift.aileron);											//10
		logValueStr(validGumstix); //blob detector									//11
		logValueStr(elevatorGumstix);												//12
		logValueStr(aileronGumstix);												//13
		logValueStr(throttleGumstix);												//14
		logValueStr((float)controllerActive); //controller							//15
		logValueStr(setpoints.altitude);  //setpoints								//16
		logValueStr(setpoints.elevator);											//17
		logValueStr(setpoints.aileron);												//18
		logValueStr((float)outputThrottle); //outputs								//19
		logValueStr((float)outputElevator);											//20
		logValueStr((float)outputAileron);											//21
		logValueStr((float)outputRudder);											//22
		logValueStr((float)RCchannel[THROTTLE]); //RC IN							//23
		logValueStr((float)RCchannel[ELEVATOR]);									//24
		logValueStr((float)RCchannel[AILERON]);										//25
		logValueStr((float)RCchannel[RUDDER]);										//26
		logValueStr((float)milisecondsTimer); //time								//27
		sprintf(str,"%d,",(int)secondsTimer);										//28
		usartBufferPutString(usart_buffer_log,str,10);								//29
		logValueStr(mpcSetpoints.elevator);											//30
		logValueStr(mpcSetpoints.aileron);											//31
		logValueStr(kalmanStates.elevator.acceleration_error);						//32
		logValueStr(kalmanStates.aileron.acceleration_error);						//33
		
				
		usartBufferPutString(usart_buffer_log,"\r\n",10);
		vTaskDelay(40);	
	}
}
