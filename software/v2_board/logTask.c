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

void logValueBin(float lValue){
	strBin=(char *) &lValue;
	*(strBin+4)='\0';
	usartBufferPutString(usart_buffer_log,str,10);
}

void logValueStr(float lValue){
	sprintf(str,"%.2f,",lValue);    
	usartBufferPutString(usart_buffer_log,str,10);		
}

void logTaskReadable(void *p){
	
	usartBufferPutString(usart_buffer_log,"Start Logging - String\r\n",10);		
	usartBufferPutString(usart_buffer_log,"Altitude,Elevator,Aileron,Blob,Setpoints,Outputs,RC IN,Time\r\n",10);
	vTaskDelay(100);	
	
	while(1){	
		logValueStr((float) (pitchAngle/10.0));
		logValueStr((float) (rollAngle/10.0));
		logValueStr(altitude.position); //altitude
		logValueStr(altitude.speed);
		logValueStr(kalmanStates.elevator.position+positionShift.elevator); //elevator
		logValueStr(kalmanStates.elevator.velocity);
		logValueStr(kalmanStates.elevator.acceleration);
		logValueStr(positionShift.elevator);		
		logValueStr(kalmanStates.aileron.position+positionShift.aileron); //aileron
		logValueStr(kalmanStates.aileron.velocity);
		logValueStr(kalmanStates.aileron.acceleration);
		logValueStr(positionShift.aileron);
		logValueStr(validGumstix); //blob detector
		logValueStr(elevatorGumstix);
		logValueStr(aileronGumstix);
		logValueStr(throttleGumstix); 
		logValueStr((float)controllerActive); //controller		
		logValueStr(altitudeTrajectory[0]);  //setpoints
		logValueStr(MPCElevatorTrajectory[0]+positionShift.elevator);
		logValueStr(MPCAileronTrajectory[0]+positionShift.aileron);
		logValueStr((float)outputThrottle); //outputs
		logValueStr((float)outputElevator);
		logValueStr((float)outputAileron);
		logValueStr((float)outputRudder);
		logValueStr((float)RCchannel[THROTTLE]); //RC IN
		logValueStr((float)RCchannel[ELEVATOR]);
		logValueStr((float)RCchannel[AILERON]);
		logValueStr((float)RCchannel[RUDDER]);
		logValueStr((float)milisecondsTimer); //time	
		sprintf(str,"%d,",secondsTimer);    
		usartBufferPutString(usart_buffer_log,str,10);
				
		usartBufferPutString(usart_buffer_log,"\r\n",10);
		vTaskDelay(40);	
	}
}

void logTaskBinary(void *p){
		vTaskDelay(1000);	
	    usartBufferPutString(usart_buffer_log,"Start Logging - Binary Float\r\n",10);	
		usartBufferPutString(usart_buffer_log,"Angles,Altitude,Elevator,Aileron,Blob,Setpoints,Outputs,RC IN,Time\r\n",10);	
		strBin=str;			
	while(1){			
		logValueBin((float) (pitchAngle/10.0)); //angles
		logValueBin((float) (rollAngle/10.0));
		logValueBin(altitude.position); //altitude
		logValueBin(altitude.speed);
		logValueBin(kalmanStates.elevator.position); //elevator
		logValueBin(kalmanStates.elevator.velocity);
		logValueBin(kalmanStates.elevator.acceleration);
		logValueBin(kalmanStates.aileron.position); //aileron
		logValueBin(kalmanStates.aileron.velocity);
		logValueBin(kalmanStates.aileron.acceleration);
		logValueBin(validGumstix); //blob detector
		logValueBin(elevatorGumstix);
		logValueBin(aileronGumstix);
		logValueBin(throttleGumstix);
		logValueBin((float)controllerActive); //controller		
		logValueBin(altitudeTrajectory[0]);  //setpoints
		logValueBin(MPCElevatorTrajectory[0]+positionShift.elevator);
		logValueBin(MPCAileronTrajectory[0]+positionShift.aileron);
		logValueBin((float)outputThrottle); //outputs
		logValueBin((float)outputElevator);
		logValueBin((float)outputAileron);
		logValueBin((float)outputRudder);
		logValueBin((float)RCchannel[THROTTLE]); //RC IN
		logValueBin((float)RCchannel[ELEVATOR]);
		logValueBin((float)RCchannel[AILERON]);		
		logValueBin((float)RCchannel[RUDDER]);	
		logValueBin((float)milisecondsTimer); //time	
		logValueBin((float)secondsTimer); 					
		usartBufferPutString(usart_buffer_log,"\r\n",10);				
	vTaskDelay(14);		
	}
}