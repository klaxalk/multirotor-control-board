#include <stdio.h>
#include "logTask.h"
#include "packets.h"
#include "controllers.h"
#include "communication.h"
#include "system.h"
#include "commTask.h"

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
		logValueStr(estimatedThrottlePos); //altitude
		logValueStr(estimatedThrottleVel);
		logValueStr(estimatedElevatorPos); //elevator
		logValueStr(estimatedElevatorVel);
		logValueStr(estimatedElevatorAcc);
		logValueStr(estimatedAileronPos); //aileron
		logValueStr(estimatedAileronVel);
		logValueStr(estimatedAileronAcc);
		logValueStr(validGumstix); //blob detector
		logValueStr(estimatedBlobElevator);
		logValueStr(estimatedBlobAileron);
		logValueStr(estimatedBlobVertical); 
		logValueStr((float)controllerActive); //controller		
		logValueStr(throttleSetpoint);  //setpoints
		logValueStr(elevatorPositionSetpoint);
		logValueStr(aileronPositionSetpoint);
		logValueStr(elevatorVelocitySetpoint);
		logValueStr(aileronVelocitySetpoint);
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
		logValueBin(estimatedThrottlePos); //altitude
		logValueBin(estimatedThrottleVel);
		logValueBin(estimatedElevatorPos); //elevator
		logValueBin(estimatedElevatorVel);
		logValueBin(estimatedElevatorAcc);
		logValueBin(estimatedAileronPos); //aileron
		logValueBin(estimatedAileronVel);
		logValueBin(estimatedAileronAcc);
		logValueBin(validGumstix); //blob detector
		logValueBin(estimatedBlobElevator);
		logValueBin(estimatedBlobAileron);
		logValueBin(estimatedBlobVertical); 
		logValueBin((float)controllerActive); //controller		
		logValueBin(throttleSetpoint); //setpoints
		logValueBin(elevatorPositionSetpoint);
		logValueBin(aileronPositionSetpoint);
		logValueBin(elevatorVelocitySetpoint);
		logValueBin(aileronVelocitySetpoint);
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